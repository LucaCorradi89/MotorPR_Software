#include <lpc17xx_libcfg_default.h>
// Automatically generated CRC function
// polynomial: 0x11021
uint16_t  CRC16_CCIT (uint8_t *data, int len)
{
	uint16_t crc=0;
    static const uint16_t table[256] = {
    0x0000U,0x1021U,0x2042U,0x3063U,0x4084U,0x50A5U,0x60C6U,0x70E7U,
    0x8108U,0x9129U,0xA14AU,0xB16BU,0xC18CU,0xD1ADU,0xE1CEU,0xF1EFU,
    0x1231U,0x0210U,0x3273U,0x2252U,0x52B5U,0x4294U,0x72F7U,0x62D6U,
    0x9339U,0x8318U,0xB37BU,0xA35AU,0xD3BDU,0xC39CU,0xF3FFU,0xE3DEU,
    0x2462U,0x3443U,0x0420U,0x1401U,0x64E6U,0x74C7U,0x44A4U,0x5485U,
    0xA56AU,0xB54BU,0x8528U,0x9509U,0xE5EEU,0xF5CFU,0xC5ACU,0xD58DU,
    0x3653U,0x2672U,0x1611U,0x0630U,0x76D7U,0x66F6U,0x5695U,0x46B4U,
    0xB75BU,0xA77AU,0x9719U,0x8738U,0xF7DFU,0xE7FEU,0xD79DU,0xC7BCU,
    0x48C4U,0x58E5U,0x6886U,0x78A7U,0x0840U,0x1861U,0x2802U,0x3823U,
    0xC9CCU,0xD9EDU,0xE98EU,0xF9AFU,0x8948U,0x9969U,0xA90AU,0xB92BU,
    0x5AF5U,0x4AD4U,0x7AB7U,0x6A96U,0x1A71U,0x0A50U,0x3A33U,0x2A12U,
    0xDBFDU,0xCBDCU,0xFBBFU,0xEB9EU,0x9B79U,0x8B58U,0xBB3BU,0xAB1AU,
    0x6CA6U,0x7C87U,0x4CE4U,0x5CC5U,0x2C22U,0x3C03U,0x0C60U,0x1C41U,
    0xEDAEU,0xFD8FU,0xCDECU,0xDDCDU,0xAD2AU,0xBD0BU,0x8D68U,0x9D49U,
    0x7E97U,0x6EB6U,0x5ED5U,0x4EF4U,0x3E13U,0x2E32U,0x1E51U,0x0E70U,
    0xFF9FU,0xEFBEU,0xDFDDU,0xCFFCU,0xBF1BU,0xAF3AU,0x9F59U,0x8F78U,
    0x9188U,0x81A9U,0xB1CAU,0xA1EBU,0xD10CU,0xC12DU,0xF14EU,0xE16FU,
    0x1080U,0x00A1U,0x30C2U,0x20E3U,0x5004U,0x4025U,0x7046U,0x6067U,
    0x83B9U,0x9398U,0xA3FBU,0xB3DAU,0xC33DU,0xD31CU,0xE37FU,0xF35EU,
    0x02B1U,0x1290U,0x22F3U,0x32D2U,0x4235U,0x5214U,0x6277U,0x7256U,
    0xB5EAU,0xA5CBU,0x95A8U,0x8589U,0xF56EU,0xE54FU,0xD52CU,0xC50DU,
    0x34E2U,0x24C3U,0x14A0U,0x0481U,0x7466U,0x6447U,0x5424U,0x4405U,
    0xA7DBU,0xB7FAU,0x8799U,0x97B8U,0xE75FU,0xF77EU,0xC71DU,0xD73CU,
    0x26D3U,0x36F2U,0x0691U,0x16B0U,0x6657U,0x7676U,0x4615U,0x5634U,
    0xD94CU,0xC96DU,0xF90EU,0xE92FU,0x99C8U,0x89E9U,0xB98AU,0xA9ABU,
    0x5844U,0x4865U,0x7806U,0x6827U,0x18C0U,0x08E1U,0x3882U,0x28A3U,
    0xCB7DU,0xDB5CU,0xEB3FU,0xFB1EU,0x8BF9U,0x9BD8U,0xABBBU,0xBB9AU,
    0x4A75U,0x5A54U,0x6A37U,0x7A16U,0x0AF1U,0x1AD0U,0x2AB3U,0x3A92U,
    0xFD2EU,0xED0FU,0xDD6CU,0xCD4DU,0xBDAAU,0xAD8BU,0x9DE8U,0x8DC9U,
    0x7C26U,0x6C07U,0x5C64U,0x4C45U,0x3CA2U,0x2C83U,0x1CE0U,0x0CC1U,
    0xEF1FU,0xFF3EU,0xCF5DU,0xDF7CU,0xAF9BU,0xBFBAU,0x8FD9U,0x9FF8U,
    0x6E17U,0x7E36U,0x4E55U,0x5E74U,0x2E93U,0x3EB2U,0x0ED1U,0x1EF0U,
    };

    while (len > 0)
    {
        crc = table[*data ^ (uint8_t)(crc >> 8)] ^ (crc << 8);
        data++;
        len--;
    }
    return crc;
}

// Automatically generated CRC function
// polynomial: 0x18005, bit reverse algorithm
uint16_t CRC16(uint8_t *data, int len)
{
	uint16_t crc=0;
    static const uint16_t table[256] = {
    0x0000U,0xC0C1U,0xC181U,0x0140U,0xC301U,0x03C0U,0x0280U,0xC241U,
    0xC601U,0x06C0U,0x0780U,0xC741U,0x0500U,0xC5C1U,0xC481U,0x0440U,
    0xCC01U,0x0CC0U,0x0D80U,0xCD41U,0x0F00U,0xCFC1U,0xCE81U,0x0E40U,
    0x0A00U,0xCAC1U,0xCB81U,0x0B40U,0xC901U,0x09C0U,0x0880U,0xC841U,
    0xD801U,0x18C0U,0x1980U,0xD941U,0x1B00U,0xDBC1U,0xDA81U,0x1A40U,
    0x1E00U,0xDEC1U,0xDF81U,0x1F40U,0xDD01U,0x1DC0U,0x1C80U,0xDC41U,
    0x1400U,0xD4C1U,0xD581U,0x1540U,0xD701U,0x17C0U,0x1680U,0xD641U,
    0xD201U,0x12C0U,0x1380U,0xD341U,0x1100U,0xD1C1U,0xD081U,0x1040U,
    0xF001U,0x30C0U,0x3180U,0xF141U,0x3300U,0xF3C1U,0xF281U,0x3240U,
    0x3600U,0xF6C1U,0xF781U,0x3740U,0xF501U,0x35C0U,0x3480U,0xF441U,
    0x3C00U,0xFCC1U,0xFD81U,0x3D40U,0xFF01U,0x3FC0U,0x3E80U,0xFE41U,
    0xFA01U,0x3AC0U,0x3B80U,0xFB41U,0x3900U,0xF9C1U,0xF881U,0x3840U,
    0x2800U,0xE8C1U,0xE981U,0x2940U,0xEB01U,0x2BC0U,0x2A80U,0xEA41U,
    0xEE01U,0x2EC0U,0x2F80U,0xEF41U,0x2D00U,0xEDC1U,0xEC81U,0x2C40U,
    0xE401U,0x24C0U,0x2580U,0xE541U,0x2700U,0xE7C1U,0xE681U,0x2640U,
    0x2200U,0xE2C1U,0xE381U,0x2340U,0xE101U,0x21C0U,0x2080U,0xE041U,
    0xA001U,0x60C0U,0x6180U,0xA141U,0x6300U,0xA3C1U,0xA281U,0x6240U,
    0x6600U,0xA6C1U,0xA781U,0x6740U,0xA501U,0x65C0U,0x6480U,0xA441U,
    0x6C00U,0xACC1U,0xAD81U,0x6D40U,0xAF01U,0x6FC0U,0x6E80U,0xAE41U,
    0xAA01U,0x6AC0U,0x6B80U,0xAB41U,0x6900U,0xA9C1U,0xA881U,0x6840U,
    0x7800U,0xB8C1U,0xB981U,0x7940U,0xBB01U,0x7BC0U,0x7A80U,0xBA41U,
    0xBE01U,0x7EC0U,0x7F80U,0xBF41U,0x7D00U,0xBDC1U,0xBC81U,0x7C40U,
    0xB401U,0x74C0U,0x7580U,0xB541U,0x7700U,0xB7C1U,0xB681U,0x7640U,
    0x7200U,0xB2C1U,0xB381U,0x7340U,0xB101U,0x71C0U,0x7080U,0xB041U,
    0x5000U,0x90C1U,0x9181U,0x5140U,0x9301U,0x53C0U,0x5280U,0x9241U,
    0x9601U,0x56C0U,0x5780U,0x9741U,0x5500U,0x95C1U,0x9481U,0x5440U,
    0x9C01U,0x5CC0U,0x5D80U,0x9D41U,0x5F00U,0x9FC1U,0x9E81U,0x5E40U,
    0x5A00U,0x9AC1U,0x9B81U,0x5B40U,0x9901U,0x59C0U,0x5880U,0x9841U,
    0x8801U,0x48C0U,0x4980U,0x8941U,0x4B00U,0x8BC1U,0x8A81U,0x4A40U,
    0x4E00U,0x8EC1U,0x8F81U,0x4F40U,0x8D01U,0x4DC0U,0x4C80U,0x8C41U,
    0x4400U,0x84C1U,0x8581U,0x4540U,0x8701U,0x47C0U,0x4680U,0x8641U,
    0x8201U,0x42C0U,0x4380U,0x8341U,0x4100U,0x81C1U,0x8081U,0x4040U,
    };

    while (len > 0)
    {
        crc = table[*data ^ (uint8_t)crc] ^ (crc >> 8);
        data++;
        len--;
    }
    return crc;
}
