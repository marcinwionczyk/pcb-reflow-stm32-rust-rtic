#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod display;

// Print panic message to probe console
use panic_probe as _;
use defmt_rtt as _;


#[rtic::app(device = pac, peripherals = true, dispatchers = [EXTI1, EXTI2])]
mod app {
    use crate::display::{TC2004ADriver};
    use embedded_hal::spi::MODE_0;
    use fugit::MicrosDurationU32;
    use stm32f4xx_hal::{
        gpio::{Edge, Input, Output, Pin,
               PA4, PA7, PA11, PA12,
               PB8,
               PC4},
        prelude::*,
        pac,
        i2c,
        timer::MonoTimerUs,
    };
    use stm32f4xx_hal::gpio::{NoPin, Speed};
    use stm32f4xx_hal::i2c::I2c;
    use stm32f4xx_hal::pac::{I2C1, SPI1, TIM5};
    use stm32f4xx_hal::spi::Spi;
    use stm32f4xx_hal::timer::DelayUs;
    use pid::*;
    use ufmt::uwrite;
    use ufmt_float::uFmt_f32;
    use heapless::String;

    #[derive(PartialEq, Clone, Copy)]
    enum SwitchEnum { None, One, Two }

    #[derive(PartialEq)]
    enum ReflowProfileEnum { LeadFree, Leaded }

    #[derive(PartialEq, Clone, Copy)]
    enum ReflowStateEnum { Idle, Preheat, Soak, Reflow, Cool, Complete, TooHot }

    #[derive(PartialEq)]
    enum ReflowStatusEnum { Off, On }

    // General Profile Constants
    const TEMPERATURE_ROOM: f32 = 50.0;
    const TEMPERATURE_SOAK_MIN: f32 = 150.0;
    const TEMPERATURE_COOL_MIN: f32 = 100.0;
    const SOAK_TEMPERATURE_STEP: f32 = 5.0;

    // Lead free profile constants
    const TEMPERATURE_SOAK_MAX_LF: f32 = 200.0;
    const TEMPERATURE_REFLOW_MAX_LF: f32 = 250.0;
    const SOAK_MICRO_PERIOD_LF: u32 = 9000;

    // Leaded profile constants
    const TEMPERATURE_SOAK_MAX_PB: f32 = 180.0;
    const TEMPERATURE_REFLOW_MAX_PB: f32 = 224.0;
    const SOAK_MICRO_PERIOD_PB: u32 = 10000;

    // PID parameters
    const PID_KP_PREHEAT: f32 = 100.0;
    const PID_KI_PREHEAT: f32 = 0.025;
    const PID_KD_PREHEAT: f32 = 20.0;
    const PID_KP_SOAK: f32 = 300.0;
    const PID_KI_SOAK: f32 = 0.05;
    const PID_KD_SOAK: f32 = 250.0;
    const PID_KP_REFLOW: f32 = 300.0;
    const PID_KI_REFLOW: f32 = 0.05;
    const PID_KD_REFLOW: f32 = 350.0;

    const LCD_MESSAGES_REFLOW_STATUS: &'static [&'static str] = &["Ready", "Preheat", "Soak", "Reflow", "Cool", "Done!", "Hot!", "Error"];

    // A monotonic timer to enable scheduling in RTIC
    #[monotonic(binds = TIM2, default = true)]
    type MilisecMono = MonoTimerUs<pac::TIM2>; // 100 Hz / 10 ms granularity

    #[shared]
    struct Shared {
        start_stop_pressed: bool,
        lf_pb_pressed: bool,
        pcf8574_interrupted: bool,
        temp: f32
    }

    // Local resources go here
    #[local]
    struct Local {
        start_stop_button: PA7<Input>,
        lf_pb_button: PC4<Input>,
        pcf8574_int_pin: PB8<Input>,
        ssr_pin: PA12<Output>,
        led_pin: PA11<Output>,
        cs_pin: PA4<Output>,
        spi: Spi<SPI1, false, u16>,
        display: TC2004ADriver<Pin<'C', 14, Output>, Pin<'C', 13, Output>, DelayUs<TIM5>, I2c<I2C1>>
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut syscfg = ctx.device.SYSCFG.constrain();
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();
        let mono = ctx.device.TIM2.monotonic_us(&clocks);
        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();
        let gpioc = ctx.device.GPIOC.split();

        let mut pcf8574_int_pin = gpiob.pb8.into_pull_up_input();
        pcf8574_int_pin.make_interrupt_source(&mut syscfg);
        pcf8574_int_pin.enable_interrupt(&mut ctx.device.EXTI);
        pcf8574_int_pin.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);

        let mut start_stop_button = gpioa.pa7.into_floating_input();
        start_stop_button.make_interrupt_source(&mut syscfg);
        start_stop_button.enable_interrupt(&mut ctx.device.EXTI);
        start_stop_button.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);

        let mut lf_pb_button = gpioc.pc4.into_floating_input();
        lf_pb_button.make_interrupt_source(&mut syscfg);
        lf_pb_button.enable_interrupt(&mut ctx.device.EXTI);
        lf_pb_button.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);

        let ssr_pin = gpioa.pa12.into_push_pull_output();
        let mut led_pin = gpioa.pa11.into_push_pull_output();
        led_pin.set_high();
        let mut cs_pin = gpioa.pa4.into_push_pull_output();
        cs_pin.set_high();

        let sck = gpioa.pa5.into_alternate::<5>().speed(Speed::VeryHigh);
        let miso = gpioa.pa6.into_alternate().speed(Speed::VeryHigh);
        let no_mosi = NoPin::new();
        let spi = ctx.device.SPI1.spi((sck, miso, no_mosi), MODE_0, 1.MHz(), &clocks).frame_size_16bit();

        let en_pin = gpioc.pc13.into_push_pull_output();
        let rs_pin = gpioc.pc14.into_push_pull_output();

        let scl = gpiob.pb6.into_alternate_open_drain();
        let sda = gpiob.pb7.into_alternate_open_drain();
        let i2c = ctx.device.I2C1.i2c( (scl, sda),
                                       i2c::Mode::Standard { frequency: 50.kHz()},
                                       &clocks);
        let delay = ctx.device.TIM5.delay_us(&clocks);
        let mut display = TC2004ADriver::new(i2c, rs_pin, en_pin, delay).build();
        display.print("Not so tiny");
        display.set_position(0, 2);
        display.print("reflow controller");
        display.home();
        main::spawn_after(2.secs()).unwrap();
        defmt::info!("init done!");
        (
            Shared {
               start_stop_pressed: false,
               lf_pb_pressed: false,
               pcf8574_interrupted: false,
               temp: 0.0
               // Initialization of shared resources go here
            },
            Local {
                start_stop_button,
                lf_pb_button,
                pcf8574_int_pin,
                ssr_pin,
                led_pin,
                cs_pin,
                spi,
                display
                // Initialization of local resources go here
            },
            init::Monotonics(mono),
        )
    }

    #[task(priority = 3, binds = EXTI4, local = [lf_pb_button], shared = [lf_pb_pressed])]
    fn lf_pb_btn_click(mut ctx: lf_pb_btn_click::Context){
        ctx.local.lf_pb_button.clear_interrupt_pending_bit();
        ctx.shared.lf_pb_pressed.lock(|lf_pb_pressed| *lf_pb_pressed = true)
    }

    #[task(priority = 3, binds = EXTI9_5, local = [start_stop_button, pcf8574_int_pin], shared = [start_stop_pressed, pcf8574_interrupted])]
    fn exti_9_5_interrupt(mut ctx: exti_9_5_interrupt::Context){
        if ctx.local.start_stop_button.check_interrupt() {
            ctx.local.start_stop_button.clear_interrupt_pending_bit();
            defmt::info!("start_stop_button pressed");
            ctx.shared.start_stop_pressed.lock(|start_stop_pressed| *start_stop_pressed = true)
        }
    }

    #[task(priority = 2, shared = [temp], local = [spi, cs_pin])]
    fn read_temp(mut ctx: read_temp::Context) {
        let mut spi_value: [u16; 1] = [0];
        defmt::info!("read temp task started");
        ctx.local.cs_pin.set_low();
        ctx.local.spi.read(&mut spi_value).unwrap();
        ctx.local.cs_pin.set_high();

        let input_is_open = (spi_value[0] & 0x4) >> 2;
        let temp_part = (spi_value[0] >> 3) as f32;
        if input_is_open == 0 {
            ctx.shared.temp.lock(|temp| *temp = temp_part / 4.0);
        }
    }

    #[task(priority = 1, shared = [temp, start_stop_pressed, lf_pb_pressed], local = [display, ssr_pin, led_pin])]
    fn main(mut ctx: main::Context) {
        let mut reflow_state = ReflowStateEnum::Idle;
        let mut reflow_profile = ReflowProfileEnum::Leaded;
        let mut reflow_status = ReflowStatusEnum::Off;
        let mut temp_local: f32 = 0.0;
        let mut set_point: f32 = 0.0;
        let mut soak_temperature_max: f32 = 0.0;
        let mut reflow_temperature_max: f32 = 0.0;
        let mut soak_micro_period: MicrosDurationU32 = 0.micros();
        let mut pid: Pid<f32> = Pid::new(0.0, 0.0);
        let mut output: ControlOutput<f32> = ControlOutput { p: 0.0, i: 0.0, d: 0.0, output: 0.0 };
        let window_size: f32 = 2000.0;
        let mut next_check = monotonics::now();
        let mut next_read = monotonics::now();
        let mut update_lcd = monotonics::now();
        let mut window_start_time = monotonics::now();
        let mut timer_soak = monotonics::now();

        loop {
            let mut switch_status = SwitchEnum::None;
            ctx.shared.start_stop_pressed.lock(|pressed| {
                if *pressed {
                    switch_status = SwitchEnum::One;
                    *pressed = false;
                    defmt::info!("start stop button pressed");
                }
            });
            ctx.shared.lf_pb_pressed.lock(|pressed| {
                if *pressed {
                    switch_status = SwitchEnum::Two;
                    *pressed = false;
                    defmt::info!("lf pb button pressed");
                }
            });
            if monotonics::now() > next_read {
                next_read += 1.secs();
                read_temp::spawn().unwrap();
                if monotonics::now() > next_check {
                    next_check += 1.secs();
                    match reflow_status {
                        ReflowStatusEnum::Off => {
                            ctx.local.led_pin.set_low()
                        }
                        ReflowStatusEnum::On => {
                            ctx.local.led_pin.toggle();
                            defmt::info!("{}, {}, {}", set_point, temp_local, output.output);
                        }
                    }
                }
                if monotonics::now() > update_lcd {
                    update_lcd += 100.millis();
                    ctx.local.display.clear();
                    ctx.local.display.print(LCD_MESSAGES_REFLOW_STATUS[reflow_state as usize]);
                    ctx.local.display.set_position(18, 0);
                    match reflow_profile {
                        ReflowProfileEnum::LeadFree => { ctx.local.display.print("LF"); }
                        ReflowProfileEnum::Leaded => { ctx.local.display.print("PB"); }
                    }
                    ctx.local.display.set_position(0, 2);
                    ctx.shared.temp.lock(|temp| {
                        temp_local = *temp;
                    });
                    let mut s: String<20> = String::new();
                    let temp_write = uFmt_f32::One(temp_local);
                    uwrite!(&mut s, "{}", temp_write).unwrap();
                    ctx.local.display.print(s.as_str());
                    ctx.local.display.write(0xDF);
                    ctx.local.display.write('C' as u8);
                }
                match reflow_state {
                    ReflowStateEnum::Idle => {
                        if temp_local >= TEMPERATURE_ROOM {
                            reflow_state = ReflowStateEnum::TooHot;
                        } else {
                            if switch_status == SwitchEnum::One {
                                defmt::info!("Setpoint, Input, Output");
                                window_start_time = monotonics::now();
                                set_point = TEMPERATURE_SOAK_MIN;
                                match reflow_profile {
                                    ReflowProfileEnum::LeadFree => {
                                        soak_temperature_max = TEMPERATURE_SOAK_MAX_LF;
                                        reflow_temperature_max = TEMPERATURE_REFLOW_MAX_LF;
                                        soak_micro_period = SOAK_MICRO_PERIOD_LF.micros();
                                    }
                                    ReflowProfileEnum::Leaded => {
                                        soak_temperature_max = TEMPERATURE_SOAK_MAX_PB;
                                        reflow_temperature_max = TEMPERATURE_REFLOW_MAX_PB;
                                        soak_micro_period = SOAK_MICRO_PERIOD_PB.micros();
                                    }
                                }
                                pid = Pid::new(set_point, window_size);
                                pid.kp = PID_KP_PREHEAT;
                                pid.kd = PID_KD_PREHEAT;
                                pid.ki = PID_KI_PREHEAT;
                                reflow_state = ReflowStateEnum::Preheat;
                            }
                        }
                    }
                    ReflowStateEnum::Preheat => {
                        reflow_status = ReflowStatusEnum::On;
                        if temp_local > TEMPERATURE_SOAK_MIN {
                            timer_soak = monotonics::now() + soak_micro_period;
                            pid.kp = PID_KP_SOAK;
                            pid.ki = PID_KI_SOAK;
                            pid.kd = PID_KD_SOAK;
                            pid.setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
                            reflow_state = ReflowStateEnum::Soak;
                        }
                    }
                    ReflowStateEnum::Soak => {
                        if monotonics::now() > timer_soak {
                            timer_soak = monotonics::now() + soak_micro_period;
                            set_point += SOAK_TEMPERATURE_STEP;
                            if set_point > soak_temperature_max {
                                pid.kp = PID_KP_REFLOW;
                                pid.ki = PID_KI_REFLOW;
                                pid.kd = PID_KD_REFLOW;
                                pid.setpoint = reflow_temperature_max;
                                reflow_state = ReflowStateEnum::Reflow;
                            }
                        }
                    }
                    ReflowStateEnum::Reflow => {
                        if temp_local >= (reflow_temperature_max - 5.0) {
                            pid.kp = PID_KP_REFLOW;
                            pid.ki = PID_KI_REFLOW;
                            pid.kd = PID_KD_REFLOW;
                            pid.setpoint = TEMPERATURE_COOL_MIN;
                            reflow_state = ReflowStateEnum::Cool;
                        }
                    }
                    ReflowStateEnum::Cool => {
                        reflow_status = ReflowStatusEnum::Off;
                        reflow_state = ReflowStateEnum::Complete;
                    }
                    ReflowStateEnum::Complete => {
                        reflow_state = ReflowStateEnum::Idle;
                    }
                    ReflowStateEnum::TooHot => {
                        if temp_local < TEMPERATURE_ROOM {
                            reflow_state = ReflowStateEnum::Idle;
                        }
                    }
                }
                match switch_status {
                    SwitchEnum::None => {}
                    SwitchEnum::One => {
                        if reflow_status == ReflowStatusEnum::On {
                            reflow_status = ReflowStatusEnum::Off;
                            reflow_state = ReflowStateEnum::Idle;
                        }
                    }
                    SwitchEnum::Two => {
                        if reflow_state == ReflowStateEnum::Idle {
                            match reflow_profile {
                                ReflowProfileEnum::LeadFree => {
                                    reflow_profile = ReflowProfileEnum::Leaded;
                                }
                                ReflowProfileEnum::Leaded => {
                                    reflow_profile = ReflowProfileEnum::LeadFree;
                                }
                            }
                        }
                    }
                }
                if reflow_status == ReflowStatusEnum::On {
                    let now = monotonics::now();
                    output = pid.next_control_output(temp_local);
                    if (now - window_start_time) > (window_size as u32).millis::<1, 1_000_000>() {
                        window_start_time += (window_size as u32).millis::<1, 1_000_000>();
                    }
                    if (output.output as u32).millis::<1, 1_000_000>() > (now - window_start_time) {
                        ctx.local.ssr_pin.set_high();
                    } else {
                        ctx.local.ssr_pin.set_low();
                    }

                }
            }
        }
    }
}
