#include <MicroNMEA.h>

// Allow debugging/regression testing under normal g++ environment.
#ifdef MICRONMEA_DEBUG
#include <stdlib.h>
#include <iostream>
using namespace std;
#endif


static long exp10(uint8_t b)
{
  long r = 1;
  while (b--)
    r *= 10;
  return r;
}


static char toHex(uint8_t nibble)
{
  if (nibble >= 10)
    return nibble + 'A' - 10;
  else
    return nibble + '0';

}


const char* MicroNMEA::skipField(const char* s)
{
  if (s == nullptr)
    return nullptr;

  while (!isEndOfFields(*s)) {
    if (*s == ',') {
      // Check next character
      if (isEndOfFields(*++s))
        break;
      else
        return s;
    }
    ++s;
  }
  return nullptr; // End of string or valid sentence
}


unsigned int MicroNMEA::parseUnsignedInt(const char *s, uint8_t len)
{
  int r = 0;
  while (len--)
    r = 10 * r + *s++ - '0';
  return r;
}


long MicroNMEA::parseFloat(const char* s, uint8_t log10Multiplier, const char** eptr, bool * resultValid)
{
  int8_t neg = 1;
  long r = 0;
  if (resultValid)
    *resultValid = false;
  while (isspace(*s))
    ++s;
  if (*s == '-') {
    neg = -1;
    ++s;
  }
  else if (*s == '+')
    ++s;

  while (isdigit(*s)) {
    r = 10 * r + *s++ - '0';
    if (resultValid)
      *resultValid = true;
  }
  r *= exp10(log10Multiplier);

  if (*s == '.') {
    ++s;
    long frac = 0;
    while (isdigit(*s) && log10Multiplier) {
      frac = 10 * frac + *s++ -'0';
      --log10Multiplier;
    }
    frac *= exp10(log10Multiplier);
    r += frac;
  }
  r *= neg; // Include effect of any minus sign

  if (eptr)
    *eptr = skipField(s);

  return r;
}


long MicroNMEA::parseDegreeMinute(const char* s, uint8_t degWidth,
                                  const char **eptr)
{
  if (*s == ',') {
    if (eptr)
      *eptr = skipField(s);
    return 0;
  }
  long r = parseUnsignedInt(s, degWidth) * 1000000L;
  s += degWidth;
  r += parseFloat(s, 6, eptr) / 60;
  return r;
}


const char* MicroNMEA::parseField(const char* s, char *result, int len)
{
  if (s == nullptr)
    return nullptr;

  int i = 0;
  while (*s != ',' && !isEndOfFields(*s)) {
    if (result && i++ < len)
      *result++ = *s;
    ++s;
  }
  if (result && i < len)
    *result = '\0'; // Terminate unless too long

  if (*s == ',')
    return ++s; // Location of start of next field
  else
    return nullptr; // End of string or valid sentence
}


const char* MicroNMEA::generateChecksum(const char* s, char* checksum)
{
  uint8_t c = 0;
  // Initial $ is omitted from checksum, if present ignore it.
  if (*s == '$')
    ++s;

  while (*s != '\0' && *s != '*')
    c ^= *s++;

  if (checksum) {
    checksum[0] = toHex(c / 16);
    checksum[1] = toHex(c % 16);
  }
  return s;
}


bool MicroNMEA::testChecksum(const char* s)
{
  char checksum[2];
  const char* p = generateChecksum(s, checksum);
  return *p == '*' && p[1] == checksum[0] && p[2] == checksum[1];
}


#ifndef MICRONMEA_DEBUG
// When debugging in normal g++ environment ostream doesn't have a
// print member function. As sendSentence() isn't needed when
// debugging don't compile it.
Stream& MicroNMEA::sendSentence(Stream& s, const char* sentence)
{
  char checksum[3];
  generateChecksum(sentence, checksum);
  checksum[2] = '\0';
  s.print(sentence);
  s.print('*');
  s.print(checksum);
  s.print("\r\n");
  return s;
}
#endif


MicroNMEA::MicroNMEA(void) :
  _talkerID('\0'),
  _messageID{0},
  _badChecksumHandler(nullptr),
  _unknownSentenceHandler(nullptr)
{
  setBuffer(nullptr, 0);
  clear();
}


MicroNMEA::MicroNMEA(void* buf, uint8_t len) :
  _talkerID('\0'),
  _messageID{0},
  _badChecksumHandler(nullptr),
  _unknownSentenceHandler(nullptr)
{
  setBuffer(buf, len);
  clear();
}


void MicroNMEA::setBuffer(void* buf, uint8_t len)
{
  _bufferLen = len;
  _buffer = (char*)buf;
  _ptr = _buffer;
  if (_bufferLen) {
    *_ptr = '\0';
    _buffer[_bufferLen - 1] = '\0';
  }
}


void MicroNMEA::clear(void)
{
  _navSystem = '\0';
  _numSat = 0;
  _hdop = 255;
  _isValid = false;
  _latitude = 999000000L;
  _longitude = 999000000L;
  _altitude = _speed = _course = LONG_MIN;
  _altitudeValid = false;
	_geoidHeightValid = false;
  _year = _month = _day = 0;
  _hour = _minute = _second = 99;
  _hundredths = 0;
}


bool MicroNMEA::process(char c)
{
  if (_buffer == nullptr || _bufferLen == 0)
    return false;
  if (c == '\0' || c == '\n' || c == '\r') {
    // Terminate buffer then reset pointer
    *_ptr = '\0';
    _ptr = _buffer;

    if (*_buffer == '$' && testChecksum(_buffer)) {
      // Valid message
      const char* data;
      if (_buffer[1] == 'G') {
        _talkerID = _buffer[2];
        data = parseField(&_buffer[3], &_messageID[0], sizeof(_messageID));
      }
      else {
        _talkerID = '\0';
        data = parseField(&_buffer[1], &_messageID[0], sizeof(_messageID));
      }

      if (data != nullptr && strcmp(&_messageID[0], "GGA") == 0)
        return processGGA(data);
      else if (data != nullptr && strcmp(&_messageID[0], "RMC") == 0)
        return processRMC(data);
      else if (_unknownSentenceHandler)
        (*_unknownSentenceHandler)(*this);
    }
    else {
      if (_badChecksumHandler && *_buffer != '\0') // don't send empty buffers as bad checksums!
        (*_badChecksumHandler)(*this);
    }
    // Return true for a complete, non-empty, sentence (even if not a valid one).
    return *_buffer != '\0'; //
  }
  else {
    *_ptr = c;
    if (_ptr < &_buffer[_bufferLen - 1])
      ++_ptr;
  }

  return false;
}


const char* MicroNMEA::parseTime(const char* s)
{
  if (*s == ',')
    return skipField(s);
  _hour = parseUnsignedInt(s, 2);
  _minute = parseUnsignedInt(s + 2, 2);
  _second = parseUnsignedInt(s + 4, 2);
  _hundredths = parseUnsignedInt(s + 7, 2);
  return skipField(s + 9);
}


const char* MicroNMEA::parseDate(const char* s)
{
  if (*s == ',')
    return skipField(s);
  _day = parseUnsignedInt(s, 2);
  _month = parseUnsignedInt(s + 2, 2);
  _year = parseUnsignedInt(s + 4, 2) + 2000;
  return skipField(s + 6);
}


bool MicroNMEA::processGGA(const char *s)
{
  // If GxGSV messages are received _talker_ID can be changed after
  // other MicroNMEA sentences. Compatibility modes can set the talker ID
  // to indicate GPS regardless of actual navigation system used.
  _navSystem = _talkerID;

  s = parseTime(s);
  if (s == nullptr)
    return false;
  // ++s;
  _latitude = parseDegreeMinute(s, 2, &s);
  if (s == nullptr)
    return false;
  if (*s == ',')
    ++s;
  else {
    if (*s == 'S')
      _latitude *= -1;
    s += 2; // Skip N/S and comma
  }
  _longitude = parseDegreeMinute(s, 3, &s);
  if (s == nullptr)
    return false;
  if (*s == ',')
    ++s;
  else {
    if (*s == 'W')
      _longitude *= -1;
    s += 2; // Skip E/W and comma
  }
  _isValid = (*s >= '1' && *s <= '5');
  s += 2; // Skip position fix flag and comma
  long tmp = parseFloat(s, 0, &s);
  _numSat = (tmp > 255 ? 255 : (tmp < 0 ? 0 : tmp));
  if (s == nullptr)
    return false;
  tmp = parseFloat(s, 1, &s);
  _hdop = (tmp > 255 || tmp < 0 ? 255 : tmp);
  if (s == nullptr)
    return false;
  bool resultValid;
  _altitude = parseFloat(s, 3, &s, &resultValid);
  if (s == nullptr)
    return false;
  if (resultValid) _altitudeValid = true;
	s += 2; // Skip M and comma
	_geoidHeight = parseFloat(s, 3, &s, &resultValid);
	if (s == nullptr)
		return false;
	if (resultValid) _geoidHeightValid = true;
  // That's all we care about
  return true;
}


bool MicroNMEA::processRMC(const char* s)
{
  // If GxGSV messages are received _talker_ID can be changed after
  // other MicroNMEA sentences. Compatibility modes can set the talker
  // ID to indicate GPS regardless of actual navigation system used.
  _navSystem = _talkerID;

  s = parseTime(s);
  if (s == nullptr)
    return false;
  _isValid = (*s == 'A');
  s += 2; // Skip validity and comma
  _latitude = parseDegreeMinute(s, 2, &s);
  if (s == nullptr)
    return false;
  if (*s == ',')
    ++s;
  else {
    if (*s == 'S')
      _latitude *= -1;
    s += 2; // Skip N/S and comma
  }
  _longitude = parseDegreeMinute(s, 3, &s);
  if (s == nullptr)
    return false;
  if (*s == ',')
    ++s;
  else {
    if (*s == 'W')
      _longitude *= -1;
    s += 2; // Skip E/W and comma
  }
  _speed = parseFloat(s, 3, &s);
  if (s == nullptr)
    return false;
  _course = parseFloat(s, 3, &s);
  if (s == nullptr)
    return false;
  s = parseDate(s);
  // That's all we care about
  return true;
}
#ifndef MICRONMEA_H
#define MICRONMEA_H

#define MICRONMEA_VERSION "2.0.6"
#include <limits.h>


/**
 * @file MicroNMEA.h
 * @author Steve Marple
 */



/**
 * @class MicroNMEA
 * @brief Process MicroNMEA sentences from GPS and GNSS receivers.
 * @details The user is responsible to allocating the buffer that MicroNMEA uses. This
 * enables a static buffer to be used if desired so that `malloc()` is not required.
 * Values returned are integers, floating-point maths is not used.
 */
class MicroNMEA {
  public:

    static const char* skipField(const char* s);
    static unsigned int parseUnsignedInt(const char* s, uint8_t len);
    static long parseFloat(const char* s, uint8_t log10Multiplier,
                           const char** eptr = nullptr, bool *resultValid = nullptr);
    static long parseDegreeMinute(const char* s, uint8_t degWidth,
                                  const char** eptr = nullptr);
    static const char* parseToComma(const char* s, char *result = nullptr,
                                    int len = 0);
    static const char* parseField(const char* s, char *result = nullptr,
                                  int len = 0);
    static const char* generateChecksum(const char* s, char* checksum);
    static bool testChecksum(const char* s);

    /**
     * @brief Send a NMEA sentence to the GNSS receiver.
     *
     * @param s Stream to which the GNSS receiver is connected
     * @param sentence The NMEA sentence to send
     * @details The sentence must start with `$`; the checksum
     * and `\r\n` terminators will be appended automatically.
     * @return The GNSS stream
     */
    static Stream& sendSentence(Stream &s, const char* sentence);

    /**
     * @brief Default constructor
     * @details User **must** call setrBuffer() before use
     */
    MicroNMEA(void);


    /**
     * @brief Construct object and pass in the buffer allocated for MicroNMEA to use
     */
    MicroNMEA(void* buffer, uint8_t len);

    /**
     * @brief Set the buffer object
     *
     * @param buf Address of the buffer
     * @param len Number of bytes allocated
     */
    void setBuffer(void* buf, uint8_t len);

    // Clear all fix information. isValid() will return false, Year,
    // month and day will all be zero. Hour, minute and second time will
    // be set to 99. Speed, course and altitude will be set to
    // LONG_MIN; the altitude validity flag will be false. Latitude and
    // longitude will be set to 999 degrees.
    /**
     * @brief Clear all fix information
     * @details `isValid()` will return false, year,
     * month and day will all be zero. Hour, minute and second will
     * be set to 99. Speed, course and altitude will be set to
     * `LONG_MIN`; the altitude validity flag will be false. Latitude and
     * longitude will be set to 999 degrees.
     */
    void clear(void);

    /**
     * @brief Get the navigation system in use
     * @details `N` = GNSS, `P` = GPS, `L` = GLONASS, `A` = Galileo, `\0` = none
     * @return char
     */
    char getNavSystem(void) const {
      return _navSystem;
    }

    /**
     * @brief Get the number of satellites in use
     *
     * @return uint8_t
     */
    uint8_t getNumSatellites(void) const {
      return _numSat;
    }

    /**
     * @brief Get the horizontal dilution of precision (HDOP), in tenths
     * @details A HDOP value of 1.1 is returned as `11`
     * @return uint8_t
     */
    uint8_t getHDOP(void) const {
      return _hdop;
    }

    /**
     * @brief Inquire if latest fix is valid
     *
     * @return true Valid
     * @return false Not valid
     */
    bool isValid(void) const {
      return _isValid;
    }

    /**
     * @brief Get the latitude, in millionths of a degree
     * @details North is positive.
     * @return long
     */
    long getLatitude(void) const {
      return _latitude;
    }

    /**
     * @brief Get the longitude, in millionths of a degree
     * @details East is positive.
     * @return long
     */
    long getLongitude(void) const {
      return _longitude;
    }

    // Altitude in millimetres.
    /**
     * @brief Get the altitude in millmetres
     *
     * @param alt Reference to long value where altitude is to be stored
     * @return true Altitude is valid
     * @return false Altitude not valid
     */
    bool getAltitude(long &alt) const {
      if (_altitudeValid)
        alt = _altitude;
      return _altitudeValid;
    }

    /**
	 * @brief Get the height above WGS84 Geoid in millimetres.
	 *
	 * @return uint16_t year
	 * @param alt Reference to long value where height is to be stored
	 * @return true Altitude is valid
	 * @return false Altitude not valid
	 */
	bool getGeoidHeight(long &alt) const {
		if (_geoidHeightValid)
			alt = _geoidHeight;
		return _geoidHeightValid;
	}

	/**
     * @brief Get the year
     *
     * @return uint16_t year
     */
    uint16_t getYear(void) const {
      return _year;
    }

    /**
     * @brief Get the month (1 - 12 inclusive)
     *
     * @return uint8_t year
     */
    uint8_t getMonth(void) const {
      return _month;
    }

    /**
     * @brief Get the day of month (1 - 31 inclusive)
     *
     * @return uint8_t month
     */
    uint8_t getDay(void) const {
      return _day;
    }

    /**
     * @brief Get the hour
     *
     * @return uint8_t hour
     */
    uint8_t getHour(void) const {
      return _hour;
    }

    /**
     * @brief Get the minute
     *
     * @return uint8_t minute
     */
    uint8_t getMinute(void) const {
      return _minute;
    }

    /**
     * @brief Get the integer part of the second
     *
     * @return uint8_t second
     */
    uint8_t getSecond(void) const {
      return _second;
    }

    /**
     * @brief Get the hundredths part of the second
     *
     * @return uint8_t hundredths
     */
    uint8_t getHundredths(void) const {
      return _hundredths;
    }

    /**
     * @brief Get the speed
     *
     * @return uint8_t speed
     */
    long getSpeed(void) const {
      return _speed;
    }

    /**
     * @brief Get the direction of travel
     * @return Direction in thousandths of a degree, clockwise from North
     */
    long getCourse(void) const {
      return _course;
    }

    /**
     * @brief Instruct MicroNMEA to process a character
     *
     * @param c Character to process
     * @return true A complete non-empty sentence has been processed (may not be valid)
     * @return false End of sentence not detected
     */
    bool process(char c);

    /**
     * @brief Register a handler to be called when bad checksums are detected
     *
     * @param handler pointer to handler function
     */
    void setBadChecksumHandler(void (*handler)(MicroNMEA& nmea)) {
      _badChecksumHandler = handler;
    }

    /**
     * @brief Register a handler to be called when an unknown NMEA sentence is detected
     *
     * @param handler pointer to handler function
     */
    void setUnknownSentenceHandler(void (*handler)(MicroNMEA& nmea)) {
      _unknownSentenceHandler = handler;
    }

    /**
     * @brief Get NMEA sentence
     *
     * @return const char*
     */
    const char* getSentence(void) const {
      return _buffer;
    }

    // Talker ID for current MicroNMEA sentence
    char getTalkerID(void) const {
      return _talkerID;
    }

    // Message ID for current MicroNMEA sentence
    const char* getMessageID(void) const {
      return (const char*)_messageID;
    }


  protected:
    static inline bool isEndOfFields(char c) {
      return c == '*' || c == '\0' || c == '\r' || c == '\n';
    }

    const char* parseTime(const char* s);
    const char* parseDate(const char* s);

    bool processGGA(const char *s);
    bool processRMC(const char* s);

  private:
    // Sentence buffer and associated pointers
    // static const uint8_t _bufferLen = 83; // 82 + NULL
    // char _buffer[_bufferLen];
    uint8_t _bufferLen;
    char* _buffer;
    char *_ptr;

    // Information from current MicroNMEA sentence
    char _talkerID;
    char _messageID[6];

    // Variables parsed and kept for user
    char _navSystem;
    bool _isValid;
    long _latitude, _longitude; // In millionths of a degree
    long _altitude; // In millimetres
    bool _altitudeValid;
	long _geoidHeight; // In millimetres
	bool _geoidHeightValid;
    long _speed, _course;
    uint16_t _year;
    uint8_t _month, _day, _hour, _minute, _second, _hundredths;
    uint8_t _numSat;
    uint8_t _hdop;

    void (*_badChecksumHandler)(MicroNMEA &nmea);
    void (*_unknownSentenceHandler)(MicroNMEA &nmea);

};


#endif