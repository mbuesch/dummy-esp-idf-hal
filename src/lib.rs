// -*- coding: utf-8 -*-

pub mod cpu {
    pub const CORES: u32 = 2;
}

pub mod peripherals {
    use super::gpio::*;
    use super::i2c::*;
    use super::modem::*;
    use super::spi::*;
    use super::uart::*;

    pub struct Peripherals {
        pub spi1: SPI1,
        pub spi2: SPI2,
        pub spi3: SPI3,
        pub i2c0: I2C0,
        pub i2c1: I2C1,
        pub uart0: UART0,
        pub uart1: UART1,
        pub uart2: UART2,
        pub modem: Modem,
        pub pins: Pins,
    }

    impl Peripherals {
        pub fn take() -> Option<Peripherals> {
            Some(Peripherals {
                spi1: SPI1(),
                spi2: SPI2(),
                spi3: SPI3(),
                i2c0: I2C0(),
                i2c1: I2C1(),
                uart0: UART0(),
                uart1: UART1(),
                uart2: UART2(),
                modem: Modem(),
                pins: Pins {
                    gpio0: Gpio0(),
                    gpio1: Gpio1(),
                    gpio3: Gpio3(),
                    gpio4: Gpio4(),
                    gpio5: Gpio5(),
                    gpio13: Gpio13(),
                    gpio14: Gpio14(),
                    gpio18: Gpio18(),
                    gpio19: Gpio19(),
                    gpio23: Gpio23(),
                    gpio25: Gpio25(),
                    gpio26: Gpio26(),
                    gpio27: Gpio27(),
                    gpio32: Gpio32(),
                    gpio33: Gpio33(),
                },
            })
        }
    }
}

pub mod peripheral {
    pub trait Peripheral {
        type P;
    }
}

pub mod gpio {
    use dummy_esp_idf_sys::EspError;
    use std::marker::PhantomData;

    pub struct AnyInputPin {}
    pub struct AnyOutputPin {}
    pub struct AnyIOPin {}
    pub struct Input {}
    pub struct Output {}

    pub enum DriveStrength {
        I5mA,
        I10mA,
        I20mA,
        I40mA,
    }

    pub trait OutputPin {
        fn downgrade_output(self) -> AnyOutputPin;
    }
    pub trait InputPin {
        fn downgrade_input(self) -> AnyInputPin;
    }
    pub trait IOPin {
        fn downgrade(self) -> AnyIOPin;
    }

    impl OutputPin for AnyOutputPin {
        fn downgrade_output(self) -> AnyOutputPin {
            self
        }
    }
    impl InputPin for AnyInputPin {
        fn downgrade_input(self) -> AnyInputPin {
            self
        }
    }
    impl IOPin for AnyIOPin {
        fn downgrade(self) -> AnyIOPin {
            self
        }
    }

    macro_rules! def_gpio {
        ($structname:ident) => {
            pub struct $structname();
            impl OutputPin for $structname {
                fn downgrade_output(self) -> AnyOutputPin {
                    AnyOutputPin {}
                }
            }
            impl InputPin for $structname {
                fn downgrade_input(self) -> AnyInputPin {
                    AnyInputPin {}
                }
            }
            impl IOPin for $structname {
                fn downgrade(self) -> AnyIOPin {
                    AnyIOPin {}
                }
            }
        };
    }

    def_gpio!(Gpio0);
    def_gpio!(Gpio1);
    def_gpio!(Gpio3);
    def_gpio!(Gpio4);
    def_gpio!(Gpio5);
    def_gpio!(Gpio13);
    def_gpio!(Gpio14);
    def_gpio!(Gpio18);
    def_gpio!(Gpio19);
    def_gpio!(Gpio23);
    def_gpio!(Gpio25);
    def_gpio!(Gpio26);
    def_gpio!(Gpio27);
    def_gpio!(Gpio32);
    def_gpio!(Gpio33);

    pub struct Pins {
        pub gpio0: Gpio0,
        pub gpio1: Gpio1,
        pub gpio3: Gpio3,
        pub gpio4: Gpio4,
        pub gpio5: Gpio5,
        pub gpio13: Gpio13,
        pub gpio14: Gpio14,
        pub gpio18: Gpio18,
        pub gpio19: Gpio19,
        pub gpio23: Gpio23,
        pub gpio25: Gpio25,
        pub gpio26: Gpio26,
        pub gpio27: Gpio27,
        pub gpio32: Gpio32,
        pub gpio33: Gpio33,
    }

    pub struct PinDriver<'a, T, MODE> {
        _p: PhantomData<(&'a T, MODE)>,
    }

    impl<'a, T, MODE> PinDriver<'a, T, MODE> {
        pub fn input(_: impl InputPin) -> Result<Self, EspError> {
            Ok(Self { _p: PhantomData })
        }

        pub fn output(_: impl OutputPin) -> Result<Self, EspError> {
            Ok(Self { _p: PhantomData })
        }

        pub fn set_high(&mut self) -> Result<(), EspError> {
            Ok(())
        }

        pub fn set_low(&mut self) -> Result<(), EspError> {
            Ok(())
        }

        pub fn toggle(&mut self) -> Result<(), EspError> {
            Ok(())
        }

        pub fn is_low(&self) -> bool {
            false
        }

        pub fn is_high(&self) -> bool {
            false
        }

        pub fn is_set_low(&self) -> bool {
            false
        }

        pub fn is_set_high(&self) -> bool {
            false
        }

        pub fn set_drive_strength(&mut self, _: DriveStrength) -> Result<(), EspError> {
            Ok(())
        }
    }
}

pub mod spi {
    macro_rules! def_spi {
        ($structname:ident) => {
            pub struct $structname();
        };
    }

    def_spi!(SPI1);
    def_spi!(SPI2);
    def_spi!(SPI3);
}

pub mod i2c {
    use super::peripheral::Peripheral;
    use dummy_esp_idf_sys::EspError;
    use std::marker::PhantomData;

    pub mod config {
        use super::super::units::*;

        #[derive(Default)]
        pub struct Config {}

        impl Config {
            pub fn new() -> Self {
                Self {}
            }

            pub fn baudrate(self, _: Hertz) -> Self {
                self
            }

            pub fn sda_enable_pullup(self, _: bool) -> Self {
                self
            }

            pub fn scl_enable_pullup(self, _: bool) -> Self {
                self
            }
        }
    }

    pub type I2cConfig = config::Config;

    pub struct I2cDriver<'a> {
        _p: PhantomData<&'a u8>,
    }

    impl<'a> I2cDriver<'a> {
        pub fn new<I2C: I2c>(
            _i2c: impl Peripheral<P = I2C>,
            _sda: impl super::gpio::IOPin,
            _scl: impl super::gpio::IOPin,
            _config: &I2cConfig,
        ) -> Result<Self, EspError> {
            Ok(Self { _p: PhantomData })
        }
    }

    impl<'a> embedded_hal_0_2::blocking::i2c::Write for I2cDriver<'a> {
        type Error = ();

        fn write(
            &mut self,
            _: embedded_hal_0_2::blocking::i2c::SevenBitAddress,
            _: &[u8],
        ) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    impl<'a> embedded_hal_0_2::blocking::i2c::WriteRead for I2cDriver<'a> {
        type Error = ();

        fn write_read(
            &mut self,
            _: embedded_hal_0_2::blocking::i2c::SevenBitAddress,
            _: &[u8],
            _: &mut [u8],
        ) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    pub trait I2c {}

    macro_rules! def_i2c {
        ($structname:ident) => {
            pub struct $structname();
            impl I2c for $structname {}
            impl Peripheral for $structname {
                type P = $structname;
            }
        };
    }

    def_i2c!(I2C0);
    def_i2c!(I2C1);
}

pub mod uart {
    use super::gpio::{InputPin, OutputPin};
    use dummy_esp_idf_sys::EspError;
    use std::marker::PhantomData;

    pub trait UART {}

    macro_rules! def_uart {
        ($structname:ident) => {
            pub struct $structname();
            impl UART for $structname {}
        };
    }

    def_uart!(UART0);
    def_uart!(UART1);
    def_uart!(UART2);

    pub mod config {
        use super::super::units::*;

        #[derive(Default)]
        pub struct Config {}

        impl Config {
            pub fn data_bits(self, _: DataBits) -> Self {
                self
            }

            pub fn parity_none(self) -> Self {
                self
            }

            pub fn parity_even(self) -> Self {
                self
            }

            pub fn parity_odd(self) -> Self {
                self
            }

            pub fn stop_bits(self, _: StopBits) -> Self {
                self
            }

            pub fn flow_control(self, _: FlowControl) -> Self {
                self
            }
        }

        pub enum DataBits {
            DataBits5,
            DataBits6,
            DataBits7,
            DataBits8,
        }

        pub enum StopBits {
            STOP1,
            STOP1P5,
            STOP2,
        }

        pub enum FlowControl {
            None,
            RTS,
            CTS,
            CTSRTS,
            MAX,
        }

        impl Config {
            pub fn baudrate(self, _baudrate: Hertz) -> Self {
                self
            }
        }
    }

    pub struct UartDriver<'a> {
        p: PhantomData<&'a u32>,
    }

    impl<'a> UartDriver<'a> {
        pub fn new(
            _uart: impl UART,
            _tx: impl OutputPin,
            _rx: impl InputPin,
            _cts: Option<impl InputPin>,
            _rts: Option<impl OutputPin>,
            _config: &config::Config,
        ) -> Result<Self, EspError> {
            Ok(Self { p: PhantomData })
        }

        pub fn split(&self) -> (UartTxDriver<'_>, UartRxDriver<'_>) {
            (
                UartTxDriver { p: PhantomData },
                UartRxDriver { p: PhantomData },
            )
        }

        pub fn read(&self, _buf: &mut [u8], _delay: u32) -> Result<usize, EspError> {
            Ok(0)
        }

        pub fn write(&self, _buf: &[u8]) -> Result<usize, EspError> {
            Ok(0)
        }
    }

    pub struct UartTxDriver<'a> {
        p: PhantomData<&'a u32>,
    }

    pub struct UartRxDriver<'a> {
        p: PhantomData<&'a u32>,
    }

    impl<'a> UartRxDriver<'a> {
        pub fn count(&self) -> Result<u8, EspError> {
            Ok(0)
        }

        pub fn flush(&self) -> Result<(), EspError> {
            Ok(())
        }
    }
}

pub mod modem {
    use super::peripheral::Peripheral;

    pub trait WifiModemPeripheral: Peripheral<P = Self> {}

    pub struct Modem();
    impl Peripheral for Modem {
        type P = Self;
    }
    impl WifiModemPeripheral for Modem {}
}

pub mod task {}

pub mod reset {
    pub fn restart() {}
}

pub mod units {
    pub struct Hertz(pub u32);
    pub struct KiloHertz(pub u32);

    impl From<KiloHertz> for Hertz {
        fn from(x: KiloHertz) -> Self {
            Hertz(x.0)
        }
    }
}

pub mod prelude {
    pub use super::peripherals::Peripherals;
    pub use super::units::*;

    pub use std::marker::PhantomData;
}

// vim: ts=4 sw=4 expandtab
