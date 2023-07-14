#include "WEH001602A.hpp"

using namespace WEH001602A;

WEH001602A_INSTANCE::WEH001602A_INSTANCE(MCP23017& port) : 
    port(port), 
    data_bus_direction(DATA_DIRECTION::DATA_IN), 
    interface_length(INTERFACE_LENGTH::EIGHT), 
    display_lines(DISPLAY_LINES::TWO), 
    character_font(CHARACTER_FONT::HEIGHT_8), 
    font_table(FONT_TABLE::ENGLISH_JAPANESE),
    cursor_direction(CURSOR_DIRECTION::INCREMENT),
    display_shift(false)
{
    write_port_configuration();
    initialise();
}

WEH001602A_INSTANCE::WEH001602A_INSTANCE(MCP23017& port, 
                INTERFACE_LENGTH interface_length,
                DISPLAY_LINES display_lines,
                CHARACTER_FONT character_font,
                FONT_TABLE font_table,
                bool cursor_visible,
                CURSOR_DIRECTION cursor_direction,
                bool cursor_blinking,
                bool display_shift) 
                :
                port(port),
                interface_length(interface_length),
                display_lines(display_lines),
                font_table(font_table),
                cursor_visible(cursor_visible),
                cursor_direction(cursor_direction),
                cursor_blinking(cursor_blinking),
                display_shift(display_shift)
{
    /* If CHARACTER_FONT::HEIGHT_10 is given for character_font when TWO display lines have been specified,
    override this value so display will still behave as programmer intended with the reduced font size */
    if(display_lines == DISPLAY_LINES::TWO)
    {
        this->character_font = CHARACTER_FONT::HEIGHT_8;
    }
    else
    {
        this->character_font = character_font;
    }

    write_port_configuration();
    initialise();
}

void WEH001602A_INSTANCE::write_port_configuration(void)
{
    // PORT MODE: 8_BIT
    port.set_port_mode(PORT_8_BIT);

    //IOCON
    port.set_ioconfig(0b10111000);

    // PORT A control pins 0, 1, 2 W/W/W rest are "don't care"
    port.set_iodir_a(0b00000000);

    // write initial port configuration
    port.write_configuration();
}

/* Customise parameters INTERFACE_LENGTH, DISPLAY_LIINES... etc. before calling this */
void WEH001602A_INSTANCE::initialise(void)
{
    // wait for busy flag to be cleared
    wait_on_busy_flag();
    
    // Send Function Set
    function_set();
    
    // Send Display On/Off Control
    wait_on_busy_flag();
    display_on_off_ctl(true);

    // Clear Display
    wait_on_busy_flag();
    clear_display();

    // Send Entry Mode Set
    wait_on_busy_flag();
    entry_mode_set();
}

bool WEH001602A_INSTANCE::read_busy_flag(void)
{
    instruction_preamble(DATA_IN, INSTRUCTION, READ);

    busy_flag = true; /* assert that device is busy until polling confirms otherwise */
    pulse_chip_enable();

    if(((port.read_input_mask(PORT_B)) & 0b10000000) == 0)
    {
        busy_flag = false;
    }
    return busy_flag;
}

bool WEH001602A_INSTANCE::wait_on_busy_flag(uint32_t ms_timeout)
{
    uint64_t start_time, current_time;    

    if(ms_timeout > 0)
    {
        uint64_t start_time = (time_us_64() / 1000);
    }

    instruction_preamble(DATA_IN, INSTRUCTION, READ);

    busy_flag = true; /* assert that device is busy until polling confirms otherwise */

    while(1)
    {
        if(ms_timeout > 0)
        {
            current_time = (time_us_64() / 1000);
        }

        pulse_chip_enable();
        sleep_ms(5);
        
        if(((port.read_input_mask(PORT_B)) & 0b10000000) == 0)
        {
            busy_flag = false;
            return busy_flag;
        }
        else if((ms_timeout > 0) && (current_time - start_time) >= ms_timeout)
        {
            return true;
        }
    }
}

void WEH001602A_INSTANCE::set_register_select(REGISTER_SELECT register_select)
{
    port.write_pin(PORT_A, RS_PIN, register_select);
}

void WEH001602A_INSTANCE::set_read_write_bit(READ_WRITE_BIT read_write)
{
    port.write_pin(PORT_A, RW_PIN, read_write);
}

void WEH001602A_INSTANCE::set_data_bus_direction(DATA_DIRECTION direction)
{
    data_bus_direction = direction;
    port.set_iodir_b(data_bus_direction);
    port.write_configuration();
}

/* Pulses CHIP_ENABLE high for (x)ms */
void WEH001602A_INSTANCE::pulse_chip_enable(void)
{
    port.write_pin(PORT_A, CE_PIN, 1);
    sleep_ms(5);                            //TODO: tune this up
    port.write_pin(PORT_A, CE_PIN, 0);
}

/* Generic procedure before any specific display command */
void WEH001602A_INSTANCE::instruction_preamble(DATA_DIRECTION data_dir = DATA_OUT, REGISTER_SELECT register_select = INSTRUCTION, READ_WRITE_BIT read_write_bit = WRITE)
{
    set_data_bus_direction(data_dir);
    set_register_select(register_select);
    set_read_write_bit(read_write_bit);

    port.set_iodir_b(0b00000000);
    port.write_configuration();
}

void WEH001602A_INSTANCE::clear_display(void)
{
    instruction_preamble();
    port.write_mask(PORT_B, CLEAR_DISPLAY);
    pulse_chip_enable();
}

void WEH001602A_INSTANCE::return_home(void)
{
    instruction_preamble();
    port.write_mask(PORT_B, RETURN_HOME);
    pulse_chip_enable();
}

void WEH001602A_INSTANCE::entry_mode_set(void)
{
    instruction_preamble();

    uint8_t cmd = ENTRY_MODE_SET;
    cmd |=  (cursor_direction << 1);
    cmd |= display_shift;

    port.write_mask(PORT_B, cmd);
    pulse_chip_enable();
}

void WEH001602A_INSTANCE::display_on_off_ctl(bool new_status)
{
    instruction_preamble();
    uint8_t cmd = DISPLAY_ON_OFF_CTL;

    cmd |= (new_status << 2);
    cmd |= (cursor_visible << 1);
    cmd |= (cursor_blinking);

    port.write_mask(PORT_B, cmd);
    pulse_chip_enable();
}

void WEH001602A_INSTANCE::function_set(void)
{
    instruction_preamble();
    uint8_t cmd = FUNCTION_SET;

    cmd |= (interface_length << 4);
    cmd |= (display_lines << 3);
    cmd |= (character_font << 2);
    cmd |= (font_table);

    port.write_mask(PORT_B, cmd);
    pulse_chip_enable();
}

void WEH001602A_INSTANCE::set_interface_length(INTERFACE_LENGTH interface_length)
{
    this->interface_length = interface_length;
}
void WEH001602A_INSTANCE::set_display_lines(DISPLAY_LINES display_lines)
{
    this->display_lines = display_lines;
}
void WEH001602A_INSTANCE::set_character_font(CHARACTER_FONT character_font)
{
    this->character_font = character_font;
}
void WEH001602A_INSTANCE::set_font_table(FONT_TABLE font_table)
{
    this->font_table = font_table;
}
void WEH001602A_INSTANCE::set_cursor_direction(CURSOR_DIRECTION cursor_direction)
{
    this->cursor_direction = cursor_direction;
}
void WEH001602A_INSTANCE::set_display_shift(bool display_shift)
{
    this->display_shift = display_shift;
}

int8_t WEH001602A_INSTANCE::set_cgram_address(uint8_t address)
{
    if(address > 0b00111111)
    {
        return 1;
    }

    instruction_preamble();
    uint8_t cmd = SET_CGRAM_ADDR;

    cmd |= (address & 0b00111111);

    port.write_mask(PORT_B, cmd);
    pulse_chip_enable();

    return 0;
}

int8_t WEH001602A_INSTANCE::set_ddram_address(uint8_t address)
{
    if(address > 0b01111111)
    {
        return 1;
    }

    instruction_preamble();
    uint8_t cmd = SET_DDRAM_ADDR;

    cmd |= (address & 0b01111111);

    port.write_mask(PORT_B, cmd);
    pulse_chip_enable();

    return 0;
}

void WEH001602A_INSTANCE::write_data(uint8_t* data, uint8_t size)
{
    instruction_preamble(DATA_OUT, DATA, WRITE);

    for(int i = 0; i < size; i++)
    {
        if(wait_on_busy_flag(25))
        {
            port.write_mask(PORT_B, data[0]);
            pulse_chip_enable();
        }
    }

}