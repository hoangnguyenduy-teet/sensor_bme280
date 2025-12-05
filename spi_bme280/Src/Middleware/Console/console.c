
/**
 * @file console.c
 * @brief Console I/O module for formatted and unformatted character input/output.
 *
 * @details
 * This module provides a lightweight interface for performing basic console I/O,
 * including:
 * - Single-character transmission (`console_putchar`) and reception (`console_getchar`)
 * - String and buffer output (`console_write`)
 * - Formatted printing similar to `printf` (`console_printf`, `console_vprintf`)
 * - Input availability check (`console_get_available_char`)
 *
 * The design is intended to be hardware-agnostic and is suitable for embedded systems.
 * Default weak implementations of low-level I/O functions allow platform-specific
 * overrides. All output is assumed to go through a UART or similar serial interface.
 *
 * Key features:
 * - Minimal runtime overhead
 * - Reentrant-safe design (caller responsibility)
 * - Support for field width and zero-padding in formatted output
 * - Graceful degradation with safe default implementations
 *
 * Design considerations:
 * - All functions assume the UART (or equivalent) has been initialized
 * - Format string parsing supports a useful subset of `printf`, including `%c`, `%d`,
 *   `%s`, `%u`, `%x`, `%p`, and `%%`
 * - Formatted printing uses internal static buffers for conversions
 *
 * @section Dependencies Dependencies
 * - `assert.h`: Used to validate pointers and preconditions
 * - `stdarg.h`: For handling variable argument lists
 * - `error_codes.h`: Defines `ERROR_OK`, `ERROR_FAIL`, etc.
 * - `console.h`: Public API declarations
 *
 * @section Configuration Configuration
 * - The application must override weak functions (`console_putchar`, `console_getchar`, etc.)
 *   if custom hardware behavior is needed
 * - Ensure the underlying serial interface is initialized before using this module
 *
 * @section Limitations Limitations
 * - Only a limited subset of `printf` format specifiers is supported
 * - No floating-point formatting
 * - Not thread-safe by default
 * - The `console_vprintf()` buffer size is limited to 16 characters at a time
 *
 * @author
 * Embedded Software Team <your.email@example.com>
 *
 * @date
 * 2025-08-02
 *
 * @version
 * 1.0.0
 *
 * @note
 * Use this module for debug logging or command-line interfaces on embedded targets.
 *
 * @warning
 * Do not call formatted output functions from interrupt context unless the underlying
 * console I/O is interrupt-safe.
 *
 * @see console.h
 */
#include "console.h"
#include "stdarg.h"
#include "error_codes.h"
#include "assert.h"

//*****************************************************************************
//
// A mapping from an integer between 0 and 15 to its ASCII character
// equivalent.
//
//*****************************************************************************
static const char * const g_pcHex = "0123456789abcdef";

/**
 * @brief Outputs a single character to the console.
 *
 * @details This function sends a single character to the console output device,
 *          typically via UART or another serial interface. It is declared as weak,
 *          allowing it to be overridden by a stronger implementation in user code.
 *          The function assumes the console hardware (e.g., UART) is initialized.
 *          If the operation is successful, it returns ERROR_OK. In case of failure,
 *          the return value may depend on the overridden implementation.
 *
 * @param c The 8-bit character to be output to the console.
 * @return uint32_t Status code indicating success (ERROR_OK) or failure .
 */
__attribute__((weak)) uint32_t console_putchar(char c)
{
	return ERROR_OK;
}

/**
 * @brief Retrieves a single character from the console.
 *
 * @details This function reads a single character from the console input device,
 *          typically via UART or another serial interface, and stores it at the
 *          memory location pointed to by the input parameter. It is declared as weak,
 *          allowing it to be overridden by a stronger implementation in user code.
 *          The function assumes the console hardware (e.g., UART) is initialized.
 *          If the operation is successful, the character is stored in *c and the
 *          function returns a success status (e.g., ERROR_OK). On failure, it returns
 *          ERROR_FAIL or an implementation-specific error code.
 *
 * @param c Pointer to an 8-bit variable where the received character will be stored.
 * @return uint32_t Status code indicating success (e.g., ERROR_OK) or failure (ERROR_FAIL).
 */

__attribute__((weak)) uint32_t console_getchar(char *c)
{
	return ERROR_FAIL;
}

/**
 * @brief Retrieves the number of available characters in the console input buffer.
 *
 * @details This function checks the console input device, typically a UART or similar
 *          serial interface, and returns the number of characters currently available
 *          in the input buffer. It is intended for use in non-blocking input operations
 *          to determine if data is ready to be read using functions like console_getchar.
 *          The function assumes the console hardware is properly initialized. The return
 *          value represents the count of available characters
 *          It is declared as weak,
 *          allowing it to be overridden by a stronger implementation in user code
 *
 * @return uint32_t The number of characters available in the console input buffer,
 *                  or 0 if the buffer is empty.
 */
__attribute__((weak)) uint32_t console_get_available_char()
{
	return 0;
}

/**
 * @brief Writes a buffer of bytes to the console one character at a time.
 *
 * This function attempts to send a sequence of bytes to the console output
 * using the `console_putchar()` function. It stops writing if a character
 * fails to be written successfully (i.e., `console_putchar()` returns
 * something other than `ERROR_OK`).
 *
 * @param pcBuf   Pointer to the buffer of bytes to write (must not be NULL).
 * @param ui32Len Number of bytes to attempt to write from the buffer.
 *
 * @return The number of bytes successfully written to the console.
 *
 * @note The function will assert if `pcBuf` is NULL.
 * @note Writing stops at the first failure of `console_putchar()`.
 */
uint32_t console_write(const char *pcBuf, uint32_t ui32Len)

{
    assert(pcBuf != 0);
    uint32_t ui32Idx;
    for (ui32Idx =0;ui32Idx<ui32Len;ui32Idx++)
    {
    	if (console_putchar(*pcBuf) == ERROR_OK) pcBuf++;
    	else break;
    }
    return ui32Idx;
}

/**
 * @brief Outputs a formatted string to console using a variable argument list.
 *
 * This function is the core implementation for UART-based formatted printing.
 * It processes the format string and variable argument list, supporting
 * a limited set of printf-style format specifiers. Output is sent via
 * console using the `console_putchar()` function.
 *
 * Supported format specifiers include:
 * - `%c` : Print a single character.
 * - `%d`, `%i` : Print a signed decimal integer.
 * - `%s` : Print a null-terminated string.
 * - `%u` : Print an unsigned decimal integer.
 * - `%x`, `%X` : Print a hexadecimal integer (using lowercase letters).
 * - `%p` : Print a pointer as a hexadecimal value.
 * - `%%` : Print a literal percent symbol.
 *
 * A field width (e.g., `%08d`, `%5s`) can be specified to pad the output with
 * spaces or zeros. Padding applies to characters, strings, integers, and
 * pointers. For example:
 * - `%8d` pads with spaces.
 * - `%08d` pads with zeros.
 *
 * @param pcString Pointer to the format string (null-terminated).
 * @param vaArgP   A va_list of arguments matching the format string.
 *
 * @note This function is typically not called directly. Instead, use
 *       `console_printf()` which wraps this function.
 *
 * @return None.
 */

void
console_vprintf(const char *pcString, va_list vaArgP)
{
    uint32_t ui32Idx, ui32Value, ui32Pos, ui32Count, ui32Base, ui32Neg;
    char *pcStr, pcBuf[16], cFill;

    //
    // Check the arguments.
    //
    assert(pcString != 0);

    //
    // Loop while there are more characters in the string.
    //
    while(*pcString)
    {
        //
        // Find the first non-% character, or the end of the string.
        //
        for(ui32Idx = 0;
            (pcString[ui32Idx] != '%') && (pcString[ui32Idx] != '\0');
            ui32Idx++)
        {
        }

        //
        // Write this portion of the string.
        //
        console_write(pcString, ui32Idx);

        //
        // Skip the portion of the string that was written.
        //
        pcString += ui32Idx;

        //
        // See if the next character is a %.
        //
        if(*pcString == '%')
        {
            //
            // Skip the %.
            //
            pcString++;

            //
            // Set the digit count to zero, and the fill character to space
            // (in other words, to the defaults).
            //
            ui32Count = 0;
            cFill = ' ';

            //
            // It may be necessary to get back here to process more characters.
            // Goto's aren't pretty, but effective.  I feel extremely dirty for
            // using not one but two of the beasts.
            //
again:

            //
            // Determine how to handle the next character.
            //
            switch(*pcString++)
            {
                //
                // Handle the digit characters.
                //
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                {
                    //
                    // If this is a zero, and it is the first digit, then the
                    // fill character is a zero instead of a space.
                    //
                    if((pcString[-1] == '0') && (ui32Count == 0))
                    {
                        cFill = '0';
                    }

                    //
                    // Update the digit count.
                    //
                    ui32Count *= 10;
                    ui32Count += pcString[-1] - '0';

                    //
                    // Get the next character.
                    //
                    goto again;
                }

                //
                // Handle the %c command.
                //
                case 'c':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui32Value = va_arg(vaArgP, uint32_t);

                    //
                    // Print out the character.
                    //
                    console_write((char *)&ui32Value, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %d and %i commands.
                //
                case 'd':
                case 'i':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui32Value = va_arg(vaArgP, uint32_t);

                    //
                    // Reset the buffer position.
                    //
                    ui32Pos = 0;

                    //
                    // If the value is negative, make it positive and indicate
                    // that a minus sign is needed.
                    //
                    if((int32_t)ui32Value < 0)
                    {
                        //
                        // Make the value positive.
                        //
                        ui32Value = -(int32_t)ui32Value;

                        //
                        // Indicate that the value is negative.
                        //
                        ui32Neg = 1;
                    }
                    else
                    {
                        //
                        // Indicate that the value is positive so that a minus
                        // sign isn't inserted.
                        //
                        ui32Neg = 0;
                    }

                    //
                    // Set the base to 10.
                    //
                    ui32Base = 10;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle the %s command.
                //
                case 's':
                {
                    //
                    // Get the string pointer from the varargs.
                    //
                    pcStr = va_arg(vaArgP, char *);

                    //
                    // Determine the length of the string.
                    //
                    for(ui32Idx = 0; pcStr[ui32Idx] != '\0'; ui32Idx++)
                    {
                    }

                    //
                    // Write the string.
                    //
                    console_write(pcStr, ui32Idx);

                    //
                    // Write any required padding spaces
                    //
                    if(ui32Count > ui32Idx)
                    {
                        ui32Count -= ui32Idx;
                        while(ui32Count--)
                        {
                        	console_write(" ", 1);
                        }
                    }

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %u command.
                //
                case 'u':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui32Value = va_arg(vaArgP, uint32_t);

                    //
                    // Reset the buffer position.
                    //
                    ui32Pos = 0;

                    //
                    // Set the base to 10.
                    //
                    ui32Base = 10;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ui32Neg = 0;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle the %x and %X commands.  Note that they are treated
                // identically; in other words, %X will use lower case letters
                // for a-f instead of the upper case letters it should use.  We
                // also alias %p to %x.
                //
                case 'x':
                case 'X':
                case 'p':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui32Value = va_arg(vaArgP, uint32_t);

                    //
                    // Reset the buffer position.
                    //
                    ui32Pos = 0;

                    //
                    // Set the base to 16.
                    //
                    ui32Base = 16;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ui32Neg = 0;

                    //
                    // Determine the number of digits in the string version of
                    // the value.
                    //
convert:
                    for(ui32Idx = 1;
                        (((ui32Idx * ui32Base) <= ui32Value) &&
                         (((ui32Idx * ui32Base) / ui32Base) == ui32Idx));
                        ui32Idx *= ui32Base, ui32Count--)
                    {
                    }

                    //
                    // If the value is negative, reduce the count of padding
                    // characters needed.
                    //
                    if(ui32Neg)
                    {
                        ui32Count--;
                    }

                    //
                    // If the value is negative and the value is padded with
                    // zeros, then place the minus sign before the padding.
                    //
                    if(ui32Neg && (cFill == '0'))
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ui32Pos++] = '-';

                        //
                        // The minus sign has been placed, so turn off the
                        // negative flag.
                        //
                        ui32Neg = 0;
                    }

                    //
                    // Provide additional padding at the beginning of the
                    // string conversion if needed.
                    //
                    if((ui32Count > 1) && (ui32Count < 16))
                    {
                        for(ui32Count--; ui32Count; ui32Count--)
                        {
                            pcBuf[ui32Pos++] = cFill;
                        }
                    }

                    //
                    // If the value is negative, then place the minus sign
                    // before the number.
                    //
                    if(ui32Neg)
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ui32Pos++] = '-';
                    }

                    //
                    // Convert the value into a string.
                    //
                    for(; ui32Idx; ui32Idx /= ui32Base)
                    {
                        pcBuf[ui32Pos++] =
                            g_pcHex[(ui32Value / ui32Idx) % ui32Base];
                    }

                    //
                    // Write the string.
                    //
                    console_write(pcBuf, ui32Pos);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %% command.
                //
                case '%':
                {
                    //
                    // Simply write a single %.
                    //
                	console_write(pcString - 1, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle all other commands.
                //
                default:
                {
                    //
                    // Indicate an error.
                    //
                	console_write("ERROR", 5);

                    //
                    // This command has been handled.
                    //
                    break;
                }
            }
        }
    }
}

//*****************************************************************************
//
//! A simple UART based printf function supporting \%c, \%d, \%p, \%s, \%u,
//! \%x, and \%X.
//!
//! \param pcString is the format string.
//! \param ... are the optional arguments, which depend on the contents of the
//! format string.
//!
//! This function is very similar to the C library <tt>fprintf()</tt> function.
//! All of its output will be sent to the UART.  Only the following formatting
//! characters are supported:
//!
//! - \%c to print a character
//! - \%d or \%i to print a decimal value
//! - \%s to print a string
//! - \%u to print an unsigned decimal value
//! - \%x to print a hexadecimal value using lower case letters
//! - \%X to print a hexadecimal value using lower case letters (not upper case
//! letters as would typically be used)
//! - \%p to print a pointer as a hexadecimal value
//! - \%\% to print out a \% character
//!
//! For \%s, \%d, \%i, \%u, \%p, \%x, and \%X, an optional number may reside
//! between the \% and the format character, which specifies the minimum number
//! of characters to use for that value; if preceded by a 0 then the extra
//! characters will be filled with zeros instead of spaces.  For example,
//! ``\%8d'' will use eight characters to print the decimal value with spaces
//! added to reach eight; ``\%08d'' will use eight characters as well but will
//! add zeroes instead of spaces.
//!
//! The type of the arguments after \e pcString must match the requirements of
//! the format string.  For example, if an integer was passed where a string
//! was expected, an error of some kind will most likely occur.
//!
//! \return None.
//
//*****************************************************************************
void
console_printf(const char *pcString, ...)
{
    va_list vaArgP;

    //
    // Start the varargs processing.
    //
    va_start(vaArgP, pcString);

    console_vprintf(pcString, vaArgP);

    //
    // We're finished with the varargs now.
    //
    va_end(vaArgP);
}
