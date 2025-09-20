// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include "rm_manager/util.hpp"

// CRC16 Table


/**
 * @brief CRC16 Caculation function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data 
 * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
 * @return : CRC16 checksum
 */
uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t ch_data;

  if (pchMessage == nullptr)
    return 0xFFFF;
  while (dwLength--)
  {
    ch_data = *pchMessage++;
    (wCRC) =
        ((uint16_t)(wCRC) >> 8) ^ W_CRC16_TABLE[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
  }

  return wCRC;
}

/**
 * @brief CRC16 Verify function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return : True or False (CRC Verify Result)
 */
uint32_t Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;

  if ((pchMessage == nullptr) || (dwLength <= 2))
    return false;

  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
  return (
      (w_expected & 0xff) == pchMessage[dwLength - 2] &&
      ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**
 * @brief Append CRC16 value to the end of the buffer
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return none
 */
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
  uint16_t w_crc = 0;

  if ((pchMessage == nullptr) || (dwLength <= 2))
    return;

  w_crc = Get_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(pchMessage), dwLength - 2, CRC16_INIT);

  pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}


uint8_t Get_CRC8_Check_Sum(const uint8_t *pchMessage, uint16_t dwLength, uint8_t ucCRC8)
{
  uint8_t ucIndex;

  while (dwLength--)
  {
    ucIndex = ucCRC8 ^ (*pchMessage++);
    ucCRC8 = W_CRC8_TABLE[ucIndex];
  }
  return ucCRC8;
}

void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
{
  uint8_t ucCRC = 0;

  if (pchMessage == 0 || dwLength <= 2)
  {
    return;
  }

  ucCRC = Get_CRC8_Check_Sum((uint8_t *)pchMessage, dwLength - 1, CRC8_INIT);

  pchMessage[dwLength - 1] = ucCRC;
}

uint32_t Verify_CRC8_Check_Sum(const uint8_t *pchMessage, uint16_t dwLength)
{
  uint8_t ucExpected = 0;

  if (pchMessage == 0 || dwLength <= 2)
  {
    return 0;
  }

  ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);

  return (ucExpected == pchMessage[dwLength - 1]);
}

std::string uint16_to_hex_string(uint16_t value) {
    std::stringstream ss;

    // 1. 设置输出为十六进制
    ss << std::hex;

    // 2. 可选：设置输出字母为大写 (A-F)
    ss << std::uppercase;

    // 3. 可选：设置宽度并用0填充，以确保输出总是4个字符 (对于uint16_t，最大FF FF)
    // std::setw(4) 设置宽度为4
    // std::setfill('0') 设置填充字符为'0'
    ss << std::setw(4) << std::setfill('0');

    // 4. 将数值写入流
    ss << value;

    // 5. 获取结果字符串
    return ss.str();
}

std::string uint16_to_hex_string_with_prefix(uint16_t value) {
    std::stringstream ss;
    ss << "0x" // 添加十六进制前缀
       << std::hex
       << std::uppercase
       << std::setw(4) << std::setfill('0')
       << value;
    return ss.str();
}