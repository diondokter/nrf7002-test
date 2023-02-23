use embedded_hal::spi::{SpiBus, SpiBusRead, SpiBusWrite, SpiDevice};

pub struct WifiBus<SPI>
where
    SPI: SpiDevice,
    SPI::Bus: SpiBus,
{
    spi: SPI,
}

impl<SPI> WifiBus<SPI>
where
    SPI: SpiDevice,
    SPI::Bus: SpiBus,
{
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    #[doc(alias = "spim_xfer_tx")]
    pub fn transfer_tx(&mut self, addres: u32, data: &[u8]) -> Result<(), SPI::Error> {
        if addres % 4 != 0 || data.len() % 4 != 0 {
            panic!("Unaligned address");
        }

        let header = [
            0x02, // PP opcode
            ((addres >> 16) & 0xFF) as u8 | 0x80,
            ((addres >> 8) & 0xFF) as u8,
            (addres & 0xFF) as u8,
        ];

        self.spi.transaction(|bus| {
            bus.write(&header)?;
            bus.write(data)?;
            Ok(())
        })
    }

    #[doc(alias = "spim_xfer_rx")]
    pub fn transfer_rx(&mut self, addres: u32, data: &mut [u8]) -> Result<(), SPI::Error> {
        if addres % 4 != 0 || data.len() % 4 != 0 {
            panic!("Unaligned address");
        }

        let header = [
            0x0B, // FASTREAD opcode
            ((addres >> 16) & 0xFF) as u8 | 0x80,
            ((addres >> 8) & 0xFF) as u8,
            (addres & 0xFF) as u8,
            0, // Dummy byte
        ];

        self.spi.transaction(|bus| {
            bus.write(&header)?;
            bus.read(data)?;
            Ok(())
        })
    }

    pub fn read_register(&mut self, register_address: u32) -> Result<u8, SPI::Error> {
        let mut tx_buffer = [register_address as u8, 0, 0, 0, 0, 0];

        self.spi.transaction(|bus| {
            bus.transfer_in_place(&mut tx_buffer)?;
            Ok(tx_buffer[1])
        })
    }

    pub fn write_register(&mut self, register_address: u32, value: u8) -> Result<(), SPI::Error> {
        let tx_buffer = [register_address as u8, value];

        self.spi.transaction(|bus| {
            bus.write(&tx_buffer)?;
            Ok(())
        })
    }
}
