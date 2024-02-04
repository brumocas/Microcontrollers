/* Copyright (c) 2022  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include <Arduino.h>
#include "commands.h"

commands_t::commands_t()
{
  count = 0;
  state = cs_wait_for_command;
  command = ' ';
  value = 0;
  process_command = NULL;

}


void commands_t::init(void (*process_command_function)(char command, float value))
{
  process_command = process_command_function;
}


void commands_t::process_char(char b)
{
  if (state == cs_wait_for_command && isalpha(b))  { // A command is allways a letter
    state = cs_reading_value;
    command = b;

  } else if (state == cs_reading_value && b == 0x0A)  { // LF (enter key received)
    // Now we can process the buffer
    if (count != 0) {
      data[count] = 0; // Null terminate the string
      value = atof((const char*)data);
    } 

    if (process_command)                  // If "process_command" is not null
      (*process_command)(command, value); // Do something with the pair (command, value)

    command = ' '; // Clear current command
    value = 0;     // Default value for "value"
    count = 0;     // Clear Buffer 
    state = cs_wait_for_command;

  } else if (state == cs_reading_value && count < COMMANDS_BUF_IN_SIZE - 1)  { // A new digit can be read
    data[count] = b;  // Store byte in the buffer
    count++;

  }
}
