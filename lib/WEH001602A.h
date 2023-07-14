//#include <pico/stdlib.h>
#include <memory>
#include "MCP23017.hpp"

namespace WEH001602A
{
    /* PORT A = control pins */
    const uint8_t CE_PIN = 0;
    const uint8_t RW_PIN = 1;
    const uint8_t RS_PIN = 2;

    /* PORT B = data pins 0-7*/

    typedef enum data_direction
    {
        DATA_OUT = 0x00,
        DATA_IN  = 0xFF
    } DATA_DIRECTION;

    typedef enum register_select
    {
        INSTRUCTION,
        DATA
    } REGISTER_SELECT;

    typedef enum read_write_bit
    {
        WRITE,
        READ
    } READ_WRITE_BIT;

    typedef enum cursor_direction
    {
        DECREMENT,
        INCREMENT
    } CURSOR_DIRECTION;

    typedef enum instruction_codes
    {
        CLEAR_DISPLAY  = 0b00000001,
        RETURN_HOME    = 0b00000010,

        // ENTRY_MODE_SET: 000001[I/D][S] <I/D = DDRAM Address Increment/Decrement upon byte write>,  <S = Display Shift direction>
        ENTRY_MODE_SET = 0b00000100,

        // DISPLAY_ON_OFF: 00001[D][C][B] <D = Display On/Off>, <C = Cursor On/Off>, <B = Blinking On/Off>
        DISPLAY_ON_OFF_CTL = 0b00001000,

        CURSOR_DISPLAY_SHIFT = 0b00010000, // 00010000 OR 00010011 see Datasheet

        // FUNCTION_SET: 0001[DL][N][F][FT1][FT0] <DL = Interface Data Length>, <N = No. of Lines>, <F = Font>, <FT(n) = Font Table bits>
        FUNCTION_SET   = 0b00100000,

        SET_CGRAM_ADDR = 0b01000000, // 01xxxxxx
        SET_DDRAM_ADDR = 0b10000000 // 1xxxxxxx
    } INSTRUCTION_CODES;

    typedef enum error_codes
    {
        INVALID_ADDRESS,
        DEVICE_BUSY
    } ERROR_CODES;

    typedef enum character_font
    {
        HEIGHT_8,
        HEIGHT_10
    } CHARACTER_FONT;

    typedef enum font_table
    {
        ENGLISH_JAPANESE = 0x00, /*DEFAULT*/
        WESTERN_EUROPEAN_1 = 0b01,
        ENGLISH_RUSSIAN = 0b10,
        WESTERN_EUROPEAN_II = 0b11
    } FONT_TABLE;

    typedef enum display_lines
    {
        ONE = 0,
        TWO = 1
    } DISPLAY_LINES;

    typedef enum interface_length
    {
        FOUR = 0x00,
        EIGHT = 0x01
    } INTERFACE_LENGTH;

    class WEH001602A_INSTANCE
    {
        private:
            MCP23017& port;
            bool busy_flag;

            //TODO: add mutators for these:
            DATA_DIRECTION data_bus_direction; // not this one
            INTERFACE_LENGTH interface_length;
            DISPLAY_LINES display_lines;
            CHARACTER_FONT character_font;
            FONT_TABLE font_table;
            CURSOR_DIRECTION cursor_direction;
            bool display_shift;
            bool cursor_visible;
            bool cursor_blinking;

            void write_port_configuration(void);
            void initialise(void);
            void instruction_preamble(DATA_DIRECTION data_dir, REGISTER_SELECT register_select, READ_WRITE_BIT read_write_bit);

        public:
            WEH001602A_INSTANCE() = delete;
            WEH001602A_INSTANCE(MCP23017& port);
            WEH001602A_INSTANCE(MCP23017& port, 
                    INTERFACE_LENGTH interface_length,
                    DISPLAY_LINES display_lines,
                    CHARACTER_FONT character_font,
                    FONT_TABLE font_table,
                    bool cursor_visible,
                    CURSOR_DIRECTION cursor_direction,
                    bool cursor_blinking,
                    bool display_shift);

            void set_data_bus_direction(DATA_DIRECTION direction);

            bool read_busy_flag(void);
            bool wait_on_busy_flag(uint32_t ms_timeout = 0);

            void pulse_chip_enable(void);
            void set_register_select(REGISTER_SELECT register_select);
            void set_read_write_bit(READ_WRITE_BIT read_write_bit);

            void clear_display(void);
            void return_home(void);
            void entry_mode_set(void);
            void display_on_off_ctl(bool new_status);
            void function_set(void);

            /* Call function_set() to update display with new values */
            void set_interface_length(INTERFACE_LENGTH interface_length);
            void set_display_lines(DISPLAY_LINES display_lines);
            void set_character_font(CHARACTER_FONT character_font);
            void set_font_table(FONT_TABLE font_table);

            /* Call display_on_off_ctl() to update display with new values */
            void set_cursor_enable(bool enabled);
            void set_cursor_blinking(bool blinking);

            /* Call entry_mode_set() to update display with new values */
            void set_cursor_direction(CURSOR_DIRECTION cursor_direction);
            void set_display_shift(bool display_shift);

            int8_t set_cgram_address(uint8_t address);
            int8_t set_ddram_address(uint8_t address);
            void write_data(uint8_t* data, uint8_t size);
    };
};