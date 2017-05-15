#ifndef _DEBUGLIB_H_
#define _DEBUGLIB_H_

/*
   Debugging and logging functions and macros
   Written for the ESP8266 but also useful for other architectures
   
   (1) The symbols: DEBUG_ERROR, DEBUG_INFO, DEBUG_LOG, DEBUG_ASSERT 
       if used, must be defined (enable output) or not defined (no output)
       before including this header file
   (2) The symbol SerialIO can be defined to direct the debug output to 
       a specific UART. If not defined, Serial will be used.
 
   Copyright 2017 Rob Redford
   This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
 */

#if !defined(SerialIO) 
   #define SerialIO Serial
#endif

#ifdef DEBUG_LOG
/* 
  Base function to print debug messages
  Requires the format string to use the PSTR() macro
   
  We need this for SAM3X becuase there is no Serial.printf function available
  While there is a Serial.printf available for the ESP8266, it is best to maintain consistency with the use of PSTR()
  
  This function should not be called directly - only through the macro below
 */
void _Log(PGM_P fmt, ...) __attribute__((format(printf, 1, 2)));

void _Log(PGM_P fmt, ...) {
   const uint8_t maxSize = 128;                        // Max resulting string size
   char          buf[maxSize]; 
   va_list       args;
    
   va_start(args, fmt);
#ifdef __SAM3X8E__
   char format[maxSize];                               // SAM3X has no vsnprintf_P so copy into a char array first
   strncpy_P(format, fmt, maxSize);
   vsnprintf(buf, maxSize, format, args);
#else
   vsnprintf_P(buf, maxSize, fmt, args);
#endif
   va_end(args);
   SerialIO.print(buf);
}

// for the LOG function, you *must* use the PSTR() macro for the format string
#define LOG(fmt, ...)           _Log(fmt, ##__VA_ARGS__)

#else 
   #define LOG(...)
#endif // LOG

#ifdef DEBUG_ASSERT
   #define ASSERT(cond, str)            \
   {                                       \
      if(!(cond)) {                        \
         SerialIO.print(__FILE__);         \
         SerialIO.print(F(" line("));      \
         SerialIO.print(__LINE__);         \
         SerialIO.print(F("): "));         \
         SerialIO.print(#cond);            \
         SerialIO.print(F(": "));          \
         SerialIO.println(str);            \
      }                                    \
   }
#else
   #define ASSERT(...)
#endif
   
#define _MSG(header, msg, value)    \
   {                                   \
      SerialIO.print(header);          \
      SerialIO.print(msg);             \
      SerialIO.print(F(": "));         \
      SerialIO.println(value);         \
   } 
   
// macros for simple messages using Serial.print. Use the F() macro for string args
#ifdef DEBUG_INFO
   #define INFO(msg, value)        _MSG(F("INFO> "), msg, value)
#else
   #define INFO(...)
#endif

#ifdef DEBUG_ERROR
   #define ERROR(msg, value)       _MSG(F("ERROR> "), msg, value)
#else
   #define ERROR(...)
#endif



#endif 