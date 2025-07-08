-- Frame Buffer Module
-- Dual-port memory for storing video frames with configurable depth
-- Supports simultaneous read/write operations for real-time video processing

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity frame_buffer is
    Generic (
        ADDR_WIDTH : integer := 20;  -- 1M pixels (1280x720 needs ~900K)
        DATA_WIDTH : integer := 24;  -- RGB 8-8-8
        MEMORY_TYPE : string := "BRAM" -- "BRAM" or "URAM" for Xilinx
    );
    Port (
        -- Clock and Reset
        clk : in STD_LOGIC;
        rst : in STD_LOGIC;
        
        -- Write Interface
        write_clk : in STD_LOGIC;
        write_addr : in STD_LOGIC_VECTOR(ADDR_WIDTH-1 downto 0);
        write_data : in STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        write_en : in STD_LOGIC;
        
        -- Read Interface
        read_clk : in STD_LOGIC;
        read_addr : in STD_LOGIC_VECTOR(ADDR_WIDTH-1 downto 0);
        read_data : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        read_en : in STD_LOGIC;
        
        -- Status
        buffer_full : out STD_LOGIC;
        buffer_empty : out STD_LOGIC
    );
end frame_buffer;

architecture Behavioral of frame_buffer is

    -- Memory array - for synthesis, this will be inferred as BRAM
    type memory_array_type is array (0 to 2**ADDR_WIDTH-1) of STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
    signal memory_array : memory_array_type := (others => (others => '0'));
    
    -- Synthesis attributes for memory inference
    attribute RAM_STYLE : string;
    attribute RAM_STYLE of memory_array : signal is MEMORY_TYPE;
    
    -- Read data registers
    signal read_data_reg : STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0) := (others => '0');
    signal read_addr_reg : STD_LOGIC_VECTOR(ADDR_WIDTH-1 downto 0) := (others => '0');
    
    -- Write pointer management
    signal write_ptr : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');
    signal read_ptr : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');
    
    -- Status signals
    signal buffer_full_reg : STD_LOGIC := '0';
    signal buffer_empty_reg : STD_LOGIC := '1';

begin

    -- Output assignments
    read_data <= read_data_reg;
    buffer_full <= buffer_full_reg;
    buffer_empty <= buffer_empty_reg;

    -- Write process
    write_process: process(write_clk)
    begin
        if rising_edge(write_clk) then
            if write_en = '1' then
                memory_array(to_integer(unsigned(write_addr))) <= write_data;
            end if;
        end if;
    end process;

    -- Read process
    read_process: process(read_clk)
    begin
        if rising_edge(read_clk) then
            if read_en = '1' then
                read_data_reg <= memory_array(to_integer(unsigned(read_addr)));
                read_addr_reg <= read_addr;
            end if;
        end if;
    end process;

    -- Status management
    status_process: process(clk, rst)
        variable write_ptr_next : unsigned(ADDR_WIDTH-1 downto 0);
        variable read_ptr_next : unsigned(ADDR_WIDTH-1 downto 0);
    begin
        if rst = '1' then
            write_ptr <= (others => '0');
            read_ptr <= (others => '0');
            buffer_full_reg <= '0';
            buffer_empty_reg <= '1';
        elsif rising_edge(clk) then
            -- Update pointers based on external address management
            if write_en = '1' then
                write_ptr <= unsigned(write_addr);
            end if;
            
            if read_en = '1' then
                read_ptr <= unsigned(read_addr);
            end if;
            
            -- Calculate status
            write_ptr_next := write_ptr + 1;
            read_ptr_next := read_ptr + 1;
            
            if write_ptr_next = read_ptr then
                buffer_full_reg <= '1';
            else
                buffer_full_reg <= '0';
            end if;
            
            if write_ptr = read_ptr then
                buffer_empty_reg <= '1';
            else
                buffer_empty_reg <= '0';
            end if;
        end if;
    end process;

end Behavioral;
