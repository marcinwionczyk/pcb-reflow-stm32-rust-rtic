use stm32f4xx_hal::hal::blocking::delay::DelayUs;
use stm32f4xx_hal::hal::blocking::i2c::{Write, WriteRead};
use stm32f4xx_hal::hal::digital::v2::OutputPin;

#[repr(u8)]
#[derive(Clone, Eq, PartialEq)]
pub enum Error {
    /// No pin RS
    RsPin = 0,
    /// No pin EN
    ENPin = 1,
    /// I2C Error
    I2C = 2,
    /// No error
    /// [Bus mode][crate::display::Mode] is invalid or not set
    InvalidMode = 3,
    /// Invalid conversion from u8 to Error
    InvalidCode = 4,
}

#[repr(u8)]
#[allow(dead_code)]
enum Command {
    ClearDisplay = 0x01,
    ReturnHome = 0x02,
    EntryModeSet = 0x04, // 1 I/D S. Assign cursor moving direction and enable shift of the entire display
    DisplayOnOffCtrl = 0x08, // 1 D C B. Set display (D), cursor (C), and blinking of cursor (B) on/off bit
    CursorOrDisplayShift = 0x10, // 1 S/C R/L - -. Set cursor moving and display shift control bit, and the direction
    SetDisplayFunc = 0x20, // 1 DL N F - -. Set interface datalength (DL: 8-bit/4-bit), numbers of display line (N: 2-line/1-line
    SetCGramAddr = 0x40,
    // LCD_SETCGRAMADDR
    SetDDRAMAddr = 0x80,   // LCD_SETDDRAMADDR
}


#[repr(u8)]
#[allow(dead_code)]
enum Move {
    Display = 0x08,
    // LCD_DISPLAYMOVE
    Cursor = 0x00,  // LCD_CURSORMOVE
}

/// Flag that controls text direction
#[repr(u8)]
pub enum Layout {
    /// Text runs from right to left
    RightToLeft = 0x00, // LCD_ENTRYRIGHT

    /// Text runs from left to right (default)
    LeftToRight = 0x02, // LCD_ENTRYLEFT
}

/// Flag that sets the display to autoscroll
#[repr(u8)]
pub enum AutoScroll {
    /// Turn AutoScroll on
    On = 0x01, // LCD_ENTRYSHIFTINCREMENT

    /// Turn AutoScroll off (default)
    Off = 0x00, // LCD_ENTRYSHIFTDECREMENT
}


/// Flag that sets the display on/off
#[repr(u8)]
pub enum Display {
    /// Turn Display on (default)
    On = 0x04, // LCD_DISPLAYON

    /// Turn Display off
    Off = 0x00, // LCD_DISPLAYOFF
}

/// Flag that sets the cursor on/off
#[repr(u8)]
pub enum Cursor {
    /// Turn Cursor on
    On = 0x02, // LCD_CURSORON

    /// Turn Cursor off
    Off = 0x00, // LCD_CURSOROFF
}

/// Flag that sets cursor background to blink
#[repr(u8)]
pub enum Blink {
    /// Turn Blink on
    On = 0x01, // LCD_BLINKON

    /// Turn Blink off (default)
    Off = 0x00, // LCD_BLINKOFF
}


/// Flag that sets backlight state
pub enum Backlight {
    /// Turn Backlight on (default)
    On,

    /// Turn Backlight off
    Off,
}

/// Flag used to indicate direction for display scrolling
#[repr(u8)]
pub enum Scroll {
    /// Scroll display right
    Right = 0x04, // LCD_MOVERIGHT

    /// Scroll display left
    Left = 0x00, // LCD_MOVELEFT
}

/// Flag for the bus mode of the display
#[repr(u8)]
pub enum Mode {
    /// Use eight-bit bus (Set by [with_full_bus][LcdDisplay::with_full_bus])
    EightBits = 0x10, // LCD_8BITMODE
}

/// Flag for the number of lines in the display
#[repr(u8)]
pub enum Lines {
    /// Use four lines if available
    ///
    /// ## Notes
    /// Since HD44780 doesn't support 4-line LCDs, 4-line display is used like a 2-line display,
    /// but half of the characters were moved below the top part. Since the interface only allows
    /// two states for amount of lines: two and one, a way to differentiate between four line and
    /// two line mode is needed. According to HHD44780 documentation, when two-line display mode is
    /// used, the bit that specifies font size is ignored. Because of that, we can use it to
    /// differentiate between four line mode and two line mode.
    FourLines = 0x0C,

    /// Use two lines if available
    TwoLines = 0x08, // LCD_2LINE

    /// Use one line (default)
    OneLine = 0x00, // LCD_1LINE
}

/// Flag for the character size of the display
#[repr(u8)]
pub enum Size {
    /// Use display with 5x10 characters
    Dots5x10 = 0x04, // LCD_5x10DOTS

    /// Use display with 5x8 characters (default)
    Dots5x8 = 0x00, // LCD_5x8DOTS
}

/// One of the most popular sizes for this kind of LCD is 16x2
const DEFAULT_COLS: u8 = 20;

const DEFAULT_DISPLAY_FUNC: u8 = Mode::EightBits as u8 | Lines::FourLines as u8;
const DEFAULT_DISPLAY_CTRL: u8 = Display::On as u8 | Cursor::Off as u8 | Blink::Off as u8;
const DEFAULT_DISPLAY_MODE: u8 = Layout::LeftToRight as u8 | AutoScroll::Off as u8;

const CMD_DELAY: u16 = 3500;
const CHR_DELAY: u16 = 450;

const I2C_ADDR: u8 = 0x20;

pub struct TC2004ADriver<RS, EN, D, I> {
    i2c: I,
    rs_pin: RS,
    en_pin: EN,
    delay: D,
    display_func: u8,
    display_mode: u8,
    display_ctrl: u8,
    offsets: [u8; 4],
}

impl<RS, EN, D, I> TC2004ADriver<RS, EN, D, I> where
    RS: OutputPin, EN: OutputPin, I: Write + WriteRead, D: DelayUs<u16> {
    pub fn new(i2c: I, rs_pin: RS, en_pin: EN, delay: D) -> Self {
        TC2004ADriver {
            i2c,
            rs_pin,
            en_pin,
            delay,
            display_func: DEFAULT_DISPLAY_FUNC,
            display_mode: DEFAULT_DISPLAY_MODE,
            display_ctrl: DEFAULT_DISPLAY_CTRL,
            offsets: [0x00, 0x40, 0x00 + DEFAULT_COLS, 0x40 + DEFAULT_COLS],
        }
    }

    pub fn send_instruction(&mut self, instruction: u8) {
        self.rs_pin.set_low().ok();
        self.delay.delay_us(50);
        self.i2c.write(I2C_ADDR, &[instruction]).ok();
        self.delay.delay_us(50);
        self.en_pin.set_high().ok();
        self.delay.delay_us(50);
        self.en_pin.set_low().ok();
    }

    pub fn send_data(&mut self, value: u8) {
        self.rs_pin.set_high().ok();
        self.delay.delay_us(50);
        self.i2c.write(I2C_ADDR, &[value]).ok();
        self.delay.delay_us(50);
        self.en_pin.set_high().ok();
        self.delay.delay_us(50);
        self.en_pin.set_low().ok();
    }

    pub fn build(mut self) -> Self {
        self.delay.delay_us(20000);
        self.en_pin.set_low().ok();
        self.rs_pin.set_low().ok();
        self.i2c.write(I2C_ADDR, &[Command::SetDisplayFunc as u8 | self.display_func]).ok();
        self.delay.delay_us(4500);
        self.i2c.write(I2C_ADDR, &[Command::SetDisplayFunc as u8 | self.display_func]).ok();
        self.delay.delay_us(150);
        self.i2c.write(I2C_ADDR, &[Command::SetDisplayFunc as u8 | self.display_func]).ok();
        self.i2c.write(I2C_ADDR, &[Command::SetDisplayFunc as u8 | self.display_func]).ok();
        self.delay.delay_us(CMD_DELAY);
        self.i2c.write(I2C_ADDR, &[Command::DisplayOnOffCtrl as u8 | self.display_ctrl]).ok();
        self.delay.delay_us(CMD_DELAY);
        self.i2c.write(I2C_ADDR, &[Command::EntryModeSet as u8 | self.display_mode]).ok();
        self.delay.delay_us(CMD_DELAY);
        self.clear();
        self.home();
        self
    }

    pub fn clear(&mut self) {
        self.send_instruction(Command::ClearDisplay as u8);
        self.delay.delay_us(CMD_DELAY);
    }

    pub fn home(&mut self) {
        self.send_instruction(Command::ReturnHome as u8);
        self.delay.delay_us(CMD_DELAY);
    }

    pub fn set_position(&mut self, col: u8, mut row: u8) {
        let max_lines = 4;
        let num_lines = 4;
        let mut pos = col;

        if row >= max_lines {
            row = max_lines.saturating_sub(1);
        }

        if row >= num_lines {
            row = num_lines.saturating_sub(1);
        }

        pos += self.offsets[row as usize];
        self.send_instruction(Command::SetDDRAMAddr as u8 | pos);
        self.delay.delay_us(CMD_DELAY);
    }

    pub fn set_scroll(&mut self, direction: Scroll, distance: u8) {
        let command = Command::CursorOrDisplayShift as u8 | Move::Display as u8 | direction as u8;
        for _ in 0..distance {
            self.send_instruction(command);
            self.delay.delay_us(CMD_DELAY);
        }
    }

    pub fn set_layout(&mut self, layout: Layout) {
        match layout {
            Layout::LeftToRight => self.display_mode |= Layout::LeftToRight as u8,
            Layout::RightToLeft => self.display_mode &= !(Layout::LeftToRight as u8),
        }
        self.send_instruction(Command::EntryModeSet as u8 | self.display_mode);
        self.delay.delay_us(CMD_DELAY);
    }

    pub fn set_display(&mut self, display: Display) {
        match display {
            Display::On => self.display_ctrl |= Display::On as u8,
            Display::Off => self.display_ctrl &= !(Display::On as u8),
        }
        self.send_instruction(Command::DisplayOnOffCtrl as u8 | self.display_ctrl);
        self.delay.delay_us(CMD_DELAY);
    }

    pub fn set_cursor(&mut self, cursor: Cursor) {
        match cursor {
            Cursor::On => self.display_ctrl |= Cursor::On as u8,
            Cursor::Off => self.display_ctrl &= !(Cursor::On as u8),
        }
        self.send_instruction(Command::DisplayOnOffCtrl as u8 | self.display_ctrl);
        self.delay.delay_us(CMD_DELAY);
    }

    pub fn set_blink(&mut self, blink: Blink) {
        match blink {
            Blink::On => self.display_ctrl |= Blink::On as u8,
            Blink::Off => self.display_ctrl &= !(Blink::On as u8),
        }
        self.send_instruction(Command::DisplayOnOffCtrl as u8 | self.display_ctrl);
        self.delay.delay_us(CMD_DELAY);
    }

    pub fn set_autoscroll(&mut self, scroll: AutoScroll) {
        match scroll {
            AutoScroll::On => self.display_mode |= AutoScroll::On as u8,
            AutoScroll::Off => self.display_mode &= !(AutoScroll::On as u8),
        }
        self.send_instruction(Command::EntryModeSet as u8 | self.display_mode);
        self.delay.delay_us(CMD_DELAY);
    }

    /// Add a new character map to the LCD memory (CGRAM) at a particular location.
    /// There are eight locations available at positions 0-7, and location values
    /// outside of this range will be bitwise masked to fall within the range, possibly
    /// overwriting an existing custom character.
    ///
    /// # Examples
    ///
    /// ```
    /// let mut lcd: LcdDisplay<_,_> = ...;
    ///
    /// // set a sideways smiley face in CGRAM at location 0.
    /// lcd.set_character(0u8,[
    ///     0b00110,
    ///     0b00001,
    ///     0b11001,
    ///     0b00001,
    ///     0b00001,
    ///     0b11001,
    ///     0b00001,
    ///     0b00110
    /// ]);
    ///
    /// // write the character code for the custom character.
    /// lcd.home();
    /// lcd.write(0u8);
    /// ```
    pub fn set_character(&mut self, mut location: u8, map: [u8; 8]) {
        location &= 0x7; // limit to locations 0-7
        self.send_instruction(Command::SetCGramAddr as u8 | (location << 3));
        for ch in map.iter() {
            self.delay.delay_us(CHR_DELAY);
            self.send_data(*ch);
        }
    }

    pub fn scroll_right(&mut self, value: u8){
        self.set_scroll(Scroll::Right, value);
    }

    pub fn scroll_left(&mut self, value: u8){
        self.set_scroll(Scroll::Left, value);
    }

    /// Set the text direction layout left-to-right. (See [set_layout][LcdDisplay::set_layout])
    ///
    /// # Examples
    ///
    /// ```
    /// let mut lcd: LcdDisplay<_,_> = ...;
    /// lcd.layout_left_to_right();
    /// ```
    pub fn layout_left_to_right(&mut self) {
        self.set_layout(Layout::LeftToRight);
    }

    /// Set the text direction layout right-to-left. (See [set_layout][LcdDisplay::set_layout])
    ///
    /// # Examples
    ///
    /// ```
    /// let mut lcd: LcdDisplay<_,_> = ...;
    /// lcd.layout_right_to_left();
    /// ```
    pub fn layout_right_to_left(&mut self) {
        self.set_layout(Layout::RightToLeft);
    }
    /// Turn the display on. (See [set_display][LcdDisplay::set_display])
    ///
    /// # Examples
    ///
    /// ```
    /// let mut lcd: LcdDisplay<_,_> = ...;
    /// lcd.display_on();
    /// ```
    pub fn display_on(&mut self) {
        self.set_display(Display::On);
    }

    /// Turn the display off. (See [set_display][LcdDisplay::set_display])
    ///
    /// # Examples
    ///
    /// ```
    /// let mut lcd: LcdDisplay<_,_> = ...;
    /// lcd.display_off();
    /// ```
    pub fn display_off(&mut self) {
        self.set_display(Display::Off);
    }

    /// Turn the cursor on. (See [set_cursor][LcdDisplay::set_cursor])
    ///
    /// # Examples
    ///
    /// ```
    /// let mut lcd: LcdDisplay<_,_> = ...;
    /// lcd.cursor_on();
    /// ```
    pub fn cursor_on(&mut self) {
        self.set_cursor(Cursor::On);
    }
    pub fn cursor_off(&mut self) {
        self.set_cursor(Cursor::Off);
    }

    /// Set the background of the cursor to blink. (See [set_blink][LcdDisplay::set_blink])
    ///
    /// # Examples
    ///
    /// ```
    /// let mut lcd: LcdDisplay<_,_> = ...;
    /// lcd.blink_on();
    /// ```
    pub fn blink_on(&mut self) {
        self.set_blink(Blink::On);
    }

    /// Set the background of the cursor to stop blinking. (See [set_blink][LcdDisplay::set_blink])
    ///
    /// # Examples
    ///
    /// ```
    /// let mut lcd: LcdDisplay<_,_> = ...;
    /// lcd.blink_off();
    /// ```
    pub fn blink_off(&mut self) {
        self.set_blink(Blink::Off);
    }

    /// Print a message to the LCD display.
    ///
    /// # Examples
    ///
    /// ```
    /// let mut lcd: LcdDisplay<_,_> = ...;
    /// lcd.print("TEST MESSAGE");
    /// ```
    pub fn print(&mut self, text: &str) {
        for ch in text.chars() {
            self.send_data(ch as u8);
        }
    }

    /// Write a single character to the LCD display.
    ///
    /// # Examples
    ///
    /// ```
    /// let mut lcd: LcdDisplay<_,_> = ...;
    /// lcd.write('A' as u8);
    /// ```
    pub fn write(&mut self, value: u8) {
        self.delay.delay_us(CHR_DELAY);
        self.send_data(value);
    }
}