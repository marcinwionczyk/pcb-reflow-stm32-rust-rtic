#![deny(unsafe_code)]
#![no_main]
#![no_std]


// Print panic message to probe console
use panic_probe as _;
use defmt_rtt as _;

const INTERVAL_MS: u32 = 500;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1, USART2])]
mod app {
    use stm32f4xx_hal::{
        gpio::{Edge, Input, PA7, PC4},
        prelude::*,
        pac,
        timer::MonoTimerUs
    };

    pub enum LfPbEnum {LF, PB}
    pub enum StartStopEnum {Start, Stop}

    // A monotonic timer to enable scheduling in RTIC
    #[monotonic(binds = TIM2, default = true)]
    type MilisecMono = MonoTimerUs<pac::TIM2>; // 100 Hz / 10 ms granularity

    #[shared]
    struct Shared {
        start_stop_status: StartStopEnum,
        lf_pb_choice: LfPbEnum
    }

    // Local resources go here
    #[local]
    struct Local {
        start_stop_button: PA7<Input>,
        lf_pb_button: PC4<Input>
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut syscfg = ctx.device.SYSCFG.constrain();
        let mut rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();
        let mono = ctx.device.TIM2.monotonic_us(&clocks);
        let gpioa = ctx.device.GPIOA.split();
        let gpioc = ctx.device.GPIOC.split();
        let mut start_stop_button = gpioa.pa7.into_floating_input();
        start_stop_button.make_interrupt_source(&mut syscfg);
        start_stop_button.enable_interrupt(&mut ctx.device.EXTI);
        start_stop_button.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);
        let mut lf_pb_button = gpioc.pc4.into_floating_input();
        lf_pb_button.make_interrupt_source(&mut syscfg);
        lf_pb_button.enable_interrupt(&mut ctx.device.EXTI);
        lf_pb_button.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);
        defmt::info!("init done!");
        (
            Shared {
               start_stop_status: StartStopEnum::Stop,
               lf_pb_choice: LfPbEnum::LF
               // Initialization of shared resources go here
            },
            Local {
                start_stop_button,
                lf_pb_button
                // Initialization of local resources go here
            },
            init::Monotonics(mono),
        )
    }

    #[task(binds = EXTI4, local = [lf_pb_button], shared = [lf_pb_choice])]
    fn lf_pb_btn_click(mut ctx: lf_pb_btn_click::Context){
        ctx.local.lf_pb_button.clear_interrupt_pending_bit();
        defmt::info!("lf_pb_button pressed");
        ctx.shared.lf_pb_choice.lock(|lf_pb_choice| {
            match lf_pb_choice {
                LfPbEnum::LF => {
                    *lf_pb_choice = LfPbEnum::PB;
                }
                LfPbEnum::PB => {
                    *lf_pb_choice = LfPbEnum::LF;
                }
            }
        })

    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }
}
