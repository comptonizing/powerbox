#include "CommandBuffer.h"

CommandBuffer &CommandBuffer::i() {
  static CommandBuffer theInstance;
  return theInstance;
}

CommandBuffer::CommandBuffer() {
  clear();
}

CommandBuffer::~CommandBuffer() {
}

void CommandBuffer::clear() {
  m_buff[0] = '\0';
  pos = m_buff;
}

const char *CommandBuffer::get() {
  return m_buff;
}

bool CommandBuffer::add(char c) {
  if ( (pos - m_buff + 1) > CMDBUFF_SIZE ) {
    return false;
  }
  *pos++ = c;
  *pos = '\0';
  return true;
}

bool CommandBuffer::verifyChecksum() {
  char *start = m_buff + 1;
  int len = pos - m_buff - 4;
  uint16_t crcRef;
  memcpy(&crcRef, start + len, 2);
  uint16_t crc = 0;
  for (int ii=0; ii<len; ii++) {
    crc = _crc16_update(crc, start[ii]);
  }
  return crc == crcRef;
}

void CommandBuffer::cut() {
  pos -= 3;
  *pos = '\0';
}

const char *CommandBuffer::getCommand() {
  cut();
  return m_buff + 1;
}
