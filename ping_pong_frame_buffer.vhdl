-- Advanced Frame Buffer with Ping-Pong Operation
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity ping_pong_frame_buffer is
    Generic (
        ADDR_WIDTH : integer := 20;
        DATA_WIDTH : integer := 24;
        FRAME_SIZE : integer := 921600  -- 1280x720
    );
    Port (
        clk : in STD_LOGIC;
        rst : in STD_LOGIC;
        
        -- Write Interface
        write_clk : in STD_LOGIC;
        write_data : in STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        write_en : in STD_LOGIC;
        frame_start_write : in STD_LOGIC;
        
        -- Read Interface  
        read_clk : in STD_LOGIC;
        read_data : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
        read_en : in STD_LOGIC;
        frame_start_read : in STD_LOGIC;
        
        -- Control
        buffer_select : out STD_LOGIC;  -- 0 = Buffer A, 1 = Buffer B
        frame_ready : out STD_LOGIC
    );
end ping_pong_frame_buffer;

architecture Behavioral of ping_pong_frame_buffer is

    -- Dual frame buffers
    component frame_buffer is
        Generic (
            ADDR_WIDTH : integer := 20;
            DATA_WIDTH : integer := 24;
            MEMORY_TYPE : string := "BRAM"
        );
        Port (
            clk : in STD_LOGIC;
            rst : in STD_LOGIC;
            write_clk : in STD_LOGIC;
            write_addr : in STD_LOGIC_VECTOR(ADDR_WIDTH-1 downto 0);
            write_data : in STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
            write_en : in STD_LOGIC;
            read_clk : in STD_LOGIC;
            read_addr : in STD_LOGIC_VECTOR(ADDR_WIDTH-1 downto 0);
            read_data : out STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);
            read_en : in STD_LOGIC;
            buffer_full : out STD_LOGIC;
            buffer_empty : out STD_LOGIC
        );
    end component;

    -- Address counters
    signal write_addr : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');
    signal read_addr : unsigned(ADDR_WIDTH-1 downto 0) := (others => '0');
    
    -- Buffer control
    signal write_buffer_sel : STD_LOGIC := '0';
    signal read_buffer_sel : STD_LOGIC := '0';
    signal frame_ready_reg : STD_LOGIC := '0';
    
    -- Buffer interface signals
    signal write_addr_a, write_addr_b : STD_LOGIC_VECTOR(ADDR_WIDTH-1 downto 0);
    signal read_addr_a, read_addr_b : STD_LOGIC_VECTOR(ADDR_WIDTH-1 downto 0);
    signal write_en_a, write_en_b : STD_LOGIC;
    signal read_en_a, read_en_b : STD_LOGIC;
    signal read_data_a, read_data_b : STD_LOGIC_VECTOR(DATA_WIDTH-1 downto 0);

begin

    buffer_select <= write_buffer_sel;
    frame_ready <= frame_ready_reg;

    -- Buffer A
    buffer_a: frame_buffer
        generic map (
            ADDR_WIDTH => ADDR_WIDTH,
            DATA_WIDTH => DATA_WIDTH,
            MEMORY_TYPE => "BRAM"
        )
        port map (
            clk => clk,
            rst => rst,
            write_clk => write_clk,
            write_addr => write_addr_a,
            write_data => write_data,
            write_en => write_en_a,
            read_clk => read_clk,
            read_addr => read_addr_a,
            read_data => read_data_a,
            read_en => read_en_a,
            buffer_full => open,
            buffer_empty => open
        );

    -- Buffer B
    buffer_b: frame_buffer
        generic map (
            ADDR_WIDTH => ADDR_WIDTH,
            DATA_WIDTH => DATA_WIDTH,
            MEMORY_TYPE => "BRAM"
        )
        port map (
            clk => clk,
            rst => rst,
            write_clk => write_clk,
            write_addr => write_addr_b,
            write_data => write_data,
            write_en => write_en_b,
            read_clk => read_clk,
            read_addr => read_addr_b,
            read_data => read_data_b,
            read_en => read_en_b,
            buffer_full => open,
            buffer_empty => open
        );

    -- Write address and control
    write_addr_a <= std_logic_vector(write_addr);
    write_addr_b <= std_logic_vector(write_addr);
    write_en_a <= write_en when write_buffer_sel = '0' else '0';
    write_en_b <= write_en when write_buffer_sel = '1' else '0';

    -- Read address and control
    read_addr_a <= std_logic_vector(read_addr);
    read_addr_b <= std_logic_vector(read_addr);
    read_en_a <= read_en when read_buffer_sel = '0' else '0';
    read_en_b <= read_en when read_buffer_sel = '1' else '0';

    -- Output mux
    read_data <= read_data_a when read_buffer_sel = '0' else read_data_b;

    -- Address generation and buffer control
    control_process: process(clk, rst)
    begin
        if rst = '1' then
            write_addr <= (others => '0');
            read_addr <= (others => '0');
            write_buffer_sel <= '0';
            read_buffer_sel <= '0';
            frame_ready_reg <= '0';
        elsif rising_edge(clk) then
            -- Write address management
            if frame_start_write = '1' then
                write_addr <= (others => '0');
                write_buffer_sel <= not write_buffer_sel;
                frame_ready_reg <= '1';
            elsif write_en = '1' then
                write_addr <= write_addr + 1;
            end if;
            
            -- Read address management
            if frame_start_read = '1' then
                read_addr <= (others => '0');
                read_buffer_sel <= not read_buffer_sel;
                frame_ready_reg <= '0';
            elsif read_en = '1' then
                read_addr <= read_addr + 1;
            end if;
        end if;
    end process;

end Behavioral;
