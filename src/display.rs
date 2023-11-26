use stm32f4xx_hal::hal::blocking::delay::DelayUs;
use stm32f4xx_hal::hal::blocking::i2c::{Write, WriteRead};
use stm32f4xx_hal::hal::digital::v2::OutputPin;


#[repr(u8)]
#[allow(dead_code)]
enum Command {
    ClearDisplay = 0x01,
    ReturnHome = 0x02,
    EntryModeSet = 0x04, // 1 I/D S. Assign cursor moving direction and enable shift of the entire display
    DisplayControl = 0x08, // 1 D C B. Set display (D), cursor (C), and blinking of cursor (B) on/off bit
    CursorShift = 0x10, // 1 S/C R/L - -. Set cursor moving and display shift control bit, and the direction
    FunctionSet = 0x20, // 1 DL N F - -. Set interface datalength (DL: 8-bit/4-bit), numbers of display line (N: 2-line/1-line
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
pub enum Entry {
    /// Text runs from right to left
    Right = 0x00, // LCD_ENTRYRIGHT

    /// Text runs from left to right (default)
    Left = 0x02, // LCD_ENTRYLEFT
}

/// Flag that sets the display to autoscroll
#[repr(u8)]
pub enum EntryShift {
    /// Turn AutoScroll on
    Increment = 0x01, // LCD_ENTRYSHIFTINCREMENT

    /// Turn AutoScroll off (default)
    Decrement = 0x00, // LCD_ENTRYSHIFTDECREMENT
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
    FourBits = 0x00,
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
    OneLine = 0x00,
    TwoLines = 0x08,
    FourLines = 0x0C,
}

#[repr(u8)]
pub enum Dots {
    FiveTimesTen = 0x04,
    FiveTimesEight = 0x00
}

/// One of the most popular sizes for this kind of LCD is 16x2
const DEFAULT_COLS: u8 = 20;

const DEFAULT_DISPLAY_FUNC: u8 = Mode::EightBits as u8 | Lines::FourLines as u8;
const DEFAULT_DISPLAY_CTRL: u8 = Display::On as u8 | Cursor::Off as u8 | Blink::Off as u8;
const DEFAULT_DISPLAY_MODE: u8 = Entry::Left as u8 | EntryShift::Decrement as u8;

const CMD_DELAY: u32 = 3500;
const CHR_DELAY: u32 = 450;
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
    num_lines: u8,
    cols: u8
}

impl<RS, EN, D, I> TC2004ADriver<RS, EN, D, I> where
    RS: OutputPin, EN: OutputPin, I: Write + WriteRead, D: DelayUs<u32>  {
    pub fn new(i2c: I, rs_pin: RS, en_pin: EN, delay_us: D) -> Self {
        TC2004ADriver {
            i2c,
            rs_pin,
            en_pin,
            delay: delay_us,
            display_func: DEFAULT_DISPLAY_FUNC,
            display_mode: DEFAULT_DISPLAY_MODE,
            display_ctrl: DEFAULT_DISPLAY_CTRL,
            offsets: [0x00, 0x40, 0x00 + DEFAULT_COLS, 0x40 + DEFAULT_COLS],
            num_lines: 4,
            cols: 20
        }
    }

    pub fn send_instruction(&mut self, instruction: u8) {
        self.rs_pin.set_low().ok();
        self.delay.delay_us(10);
        self.i2c.write(I2C_ADDR, &[instruction]).ok();
        self.delay.delay_us(10);
        self.en_pin.set_high().ok();
        self.delay.delay_us(10);
        self.en_pin.set_low().ok();
    }

    pub fn send_data(&mut self, value: u8) {
        self.rs_pin.set_high().ok();
        self.delay.delay_us(10);
        self.i2c.write(I2C_ADDR, &[value]).ok();
        self.delay.delay_us(10);
        self.en_pin.set_high().ok();
        self.delay.delay_us(10);
        self.en_pin.set_low().ok();
    }

    pub fn build(mut self) -> Self {
        self.delay.delay_us(100000);
        self.rs_pin.set_low().ok();
        self.en_pin.set_low().ok();
        // this is according to the hitachi HD44780 datasheet
        // page 45 figure 23

        // Send function set command sequence
        self.i2c.write(I2C_ADDR, &[Command::FunctionSet as u8 | self.display_func]).ok();
        self.delay.delay_us(4500);  // wait for mare than 4.1 ms
        // second try
        self.i2c.write(I2C_ADDR, &[Command::FunctionSet as u8 | self.display_func]).ok();
        self.delay.delay_us(150);
        // third go
        self.i2c.write(I2C_ADDR, &[Command::FunctionSet as u8 | self.display_func]).ok();
        self.delay.delay_us(150);
        // finally set
        self.i2c.write(I2C_ADDR, &[Command::FunctionSet as u8 | self.display_func]).ok();
        self.delay.delay_us(60);
        self.display_on();
        self.clear();
        self.home();
        defmt::info!("display initiated");
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
        let command = Command::CursorShift as u8 | Move::Display as u8 | direction as u8;
        for _ in 0..distance {
            self.send_instruction(command);
            self.delay.delay_us(CMD_DELAY);
        }
    }

    pub fn set_layout(&mut self, layout: Entry) {
        match layout {
            Entry::Left => self.display_mode |= Entry::Left as u8,
            Entry::Right => self.display_mode &= !(Entry::Left as u8),
        }
        self.send_instruction(Command::EntryModeSet as u8 | self.display_mode);
        self.delay.delay_us(CMD_DELAY);
    }

    pub fn set_display(&mut self, display: Display) {
        match display {
            Display::On => self.display_ctrl |= Display::On as u8,
            Display::Off => self.display_ctrl &= !(Display::On as u8),
        }
        self.send_instruction(Command::DisplayControl as u8 | self.display_ctrl);
        self.delay.delay_us(CMD_DELAY);
    }

    pub fn set_cursor(&mut self, cursor: Cursor) {
        match cursor {
            Cursor::On => self.display_ctrl |= Cursor::On as u8,
            Cursor::Off => self.display_ctrl &= !(Cursor::On as u8),
        }
        self.send_instruction(Command::DisplayControl as u8 | self.display_ctrl);
        self.delay.delay_us(CMD_DELAY);
    }

    pub fn set_blink(&mut self, blink: Blink) {
        match blink {
            Blink::On => self.display_ctrl |= Blink::On as u8,
            Blink::Off => self.display_ctrl &= !(Blink::On as u8),
        }
        self.send_instruction(Command::DisplayControl as u8 | self.display_ctrl);
        self.delay.delay_us(CMD_DELAY);
    }

    pub fn set_autoscroll(&mut self, scroll: EntryShift) {
        match scroll {
            EntryShift::Increment => self.display_mode |= EntryShift::Increment as u8,
            EntryShift::Decrement => self.display_mode &= !(EntryShift::Increment as u8),
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
        self.set_layout(Entry::Left);
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
        self.set_layout(Entry::Right);
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