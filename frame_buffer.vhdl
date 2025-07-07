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

-- Video Output Module
-- Generates video timing and applies stabilization offsets
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity video_output is
    Generic (
        -- Video timing parameters (1280x720@60Hz)
        H_ACTIVE : integer := 1280;
        V_ACTIVE : integer := 720;
        H_FRONT_PORCH : integer := 110;
        H_SYNC_WIDTH : integer := 40;
        H_BACK_PORCH : integer := 220;
        V_FRONT_PORCH : integer := 5;
        V_SYNC_WIDTH : integer := 5;
        V_BACK_PORCH : integer := 20;
        
        -- Stabilization parameters
        MAX_OFFSET : integer := 128  -- Maximum stabilization offset
    );
    Port (
        clk : in STD_LOGIC;
        rst : in STD_LOGIC;
        
        -- Input video stream
        pixel_clk : in STD_LOGIC;
        frame_data : in STD_LOGIC_VECTOR(23 downto 0);
        pixel_valid : in STD_LOGIC;
        
        -- Stabilization control
        offset_x : in STD_LOGIC_VECTOR(15 downto 0);
        offset_y : in STD_LOGIC_VECTOR(15 downto 0);
        offset_valid : in STD_LOGIC;
        stabilization_enable : in STD_LOGIC := '1';
        
        -- HDMI Output
        hdmi_tx_clk : out STD_LOGIC;
        hdmi_tx_data : out STD_LOGIC_VECTOR(23 downto 0);
        hdmi_tx_de : out STD_LOGIC;
        hdmi_tx_hsync : out STD_LOGIC;
        hdmi_tx_vsync : out STD_LOGIC;
        
        -- Status
        output_active : out STD_LOGIC;
        frame_count : out STD_LOGIC_VECTOR(15 downto 0)
    );
end video_output;

architecture Behavioral of video_output is

    -- Video timing constants
    constant H_TOTAL : integer := H_ACTIVE + H_FRONT_PORCH + H_SYNC_WIDTH + H_BACK_PORCH;
    constant V_TOTAL : integer := V_ACTIVE + V_FRONT_PORCH + V_SYNC_WIDTH + V_BACK_PORCH;
    constant H_SYNC_START : integer := H_ACTIVE + H_FRONT_PORCH;
    constant H_SYNC_END : integer := H_ACTIVE + H_FRONT_PORCH + H_SYNC_WIDTH;
    constant V_SYNC_START : integer := V_ACTIVE + V_FRONT_PORCH;
    constant V_SYNC_END : integer := V_ACTIVE + V_FRONT_PORCH + V_SYNC_WIDTH;

    -- Timing counters
    signal h_counter : unsigned(11 downto 0) := (others => '0');
    signal v_counter : unsigned(11 downto 0) := (others => '0');
    
    -- Stabilization offsets
    signal offset_x_reg : signed(15 downto 0) := (others => '0');
    signal offset_y_reg : signed(15 downto 0) := (others => '0');
    signal offset_valid_reg : STD_LOGIC := '0';
    
    -- Adjusted coordinates
    signal adj_x : signed(15 downto 0);
    signal adj_y : signed(15 downto 0);
    signal coord_valid : STD_LOGIC;
    
    -- Output registers
    signal hdmi_data_reg : STD_LOGIC_VECTOR(23 downto 0) := (others => '0');
    signal hdmi_de_reg : STD_LOGIC := '0';
    signal hdmi_hsync_reg : STD_LOGIC := '0';
    signal hdmi_vsync_reg : STD_LOGIC := '0';
    
    -- Frame counter
    signal frame_count_reg : unsigned(15 downto 0) := (others => '0');
    
    -- Pipeline registers for timing
    signal h_active_area : STD_LOGIC;
    signal v_active_area : STD_LOGIC;
    signal video_active : STD_LOGIC;
    signal hsync_pulse : STD_LOGIC;
    signal vsync_pulse : STD_LOGIC;
    
    -- Border handling
    signal border_color : STD_LOGIC_VECTOR(23 downto 0) := x"404040"; -- Dark gray
    signal use_border : STD_LOGIC;

begin

    -- Output assignments
    hdmi_tx_clk <= pixel_clk;
    hdmi_tx_data <= hdmi_data_reg;
    hdmi_tx_de <= hdmi_de_reg;
    hdmi_tx_hsync <= hdmi_hsync_reg;
    hdmi_tx_vsync <= hdmi_vsync_reg;
    output_active <= video_active;
    frame_count <= std_logic_vector(frame_count_reg);

    -- Video timing generation
    timing_generator: process(pixel_clk, rst)
    begin
        if rst = '1' then
            h_counter <= (others => '0');
            v_counter <= (others => '0');
            frame_count_reg <= (others => '0');
        elsif rising_edge(pixel_clk) then
            -- Horizontal counter
            if h_counter = H_TOTAL - 1 then
                h_counter <= (others => '0');
                
                -- Vertical counter
                if v_counter = V_TOTAL - 1 then
                    v_counter <= (others => '0');
                    frame_count_reg <= frame_count_reg + 1;
                else
                    v_counter <= v_counter + 1;
                end if;
            else
                h_counter <= h_counter + 1;
            end if;
        end if;
    end process;

    -- Generate control signals
    h_active_area <= '1' when h_counter < H_ACTIVE else '0';
    v_active_area <= '1' when v_counter < V_ACTIVE else '0';
    video_active <= h_active_area and v_active_area;
    
    hsync_pulse <= '1' when (h_counter >= H_SYNC_START and h_counter < H_SYNC_END) else '0';
    vsync_pulse <= '1' when (v_counter >= V_SYNC_START and v_counter < V_SYNC_END) else '0';

    -- Stabilization offset management
    offset_management: process(pixel_clk, rst)
    begin
        if rst = '1' then
            offset_x_reg <= (others => '0');
            offset_y_reg <= (others => '0');
            offset_valid_reg <= '0';
        elsif rising_edge(pixel_clk) then
            if offset_valid = '1' and stabilization_enable = '1' then
                -- Limit offsets to prevent excessive movement
                if signed(offset_x) > MAX_OFFSET then
                    offset_x_reg <= to_signed(MAX_OFFSET, 16);
                elsif signed(offset_x) < -MAX_OFFSET then
                    offset_x_reg <= to_signed(-MAX_OFFSET, 16);
                else
                    offset_x_reg <= signed(offset_x);
                end if;
                
                if signed(offset_y) > MAX_OFFSET then
                    offset_y_reg <= to_signed(MAX_OFFSET, 16);
                elsif signed(offset_y) < -MAX_OFFSET then
                    offset_y_reg <= to_signed(-MAX_OFFSET, 16);
                else
                    offset_y_reg <= signed(offset_y);
                end if;
                
                offset_valid_reg <= '1';
            end if;
        end if;
    end process;

    -- Calculate adjusted coordinates
    coordinate_adjustment: process(pixel_clk)
        variable temp_x : signed(15 downto 0);
        variable temp_y : signed(15 downto 0);
    begin
        if rising_edge(pixel_clk) then
            if stabilization_enable = '1' and offset_valid_reg = '1' then
                temp_x := signed(resize(h_counter, 16)) + offset_x_reg;
                temp_y := signed(resize(v_counter, 16)) + offset_y_reg;
            else
                temp_x := signed(resize(h_counter, 16));
                temp_y := signed(resize(v_counter, 16));
            end if;
            
            adj_x <= temp_x;
            adj_y <= temp_y;
            
            -- Check if adjusted coordinates are within valid range
            if temp_x >= 0 and temp_x < H_ACTIVE and 
               temp_y >= 0 and temp_y < V_ACTIVE then
                coord_valid <= '1';
            else
                coord_valid <= '0';
            end if;
        end if;
    end process;

    -- Determine if border should be used
    use_border <= not coord_valid when stabilization_enable = '1' else '0';

    -- Video output pipeline
    video_pipeline: process(pixel_clk, rst)
    begin
        if rst = '1' then
            hdmi_data_reg <= (others => '0');
            hdmi_de_reg <= '0';
            hdmi_hsync_reg <= '0';
            hdmi_vsync_reg <= '0';
        elsif rising_edge(pixel_clk) then
            -- Data output
            if video_active = '1' then
                if use_border = '1' then
                    hdmi_data_reg <= border_color;
                else
                    hdmi_data_reg <= frame_data;
                end if;
            else
                hdmi_data_reg <= (others => '0');
            end if;
            
            -- Control signals
            hdmi_de_reg <= video_active;
            hdmi_hsync_reg <= hsync_pulse;
            hdmi_vsync_reg <= vsync_pulse;
        end if;
    end process;

end Behavioral;

-- Advanced Video Output with Bilinear Interpolation
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity video_output_interpolated is
    Generic (
        H_ACTIVE : integer := 1280;
        V_ACTIVE : integer := 720;
        H_FRONT_PORCH : integer := 110;
        H_SYNC_WIDTH : integer := 40;
        H_BACK_PORCH : integer := 220;
        V_FRONT_PORCH : integer := 5;
        V_SYNC_WIDTH : integer := 5;
        V_BACK_PORCH : integer := 20;
        INTERPOLATION_BITS : integer := 8  -- Sub-pixel precision
    );
    Port (
        clk : in STD_LOGIC;
        rst : in STD_LOGIC;
        pixel_clk : in STD_LOGIC;
        
        -- Frame buffer interface
        frame_addr : out STD_LOGIC_VECTOR(19 downto 0);
        frame_data : in STD_LOGIC_VECTOR(23 downto 0);
        frame_req : out STD_LOGIC;
        
        -- Stabilization control
        offset_x : in STD_LOGIC_VECTOR(15 downto 0);
        offset_y : in STD_LOGIC_VECTOR(15 downto 0);
        offset_valid : in STD_LOGIC;
        
        -- HDMI Output
        hdmi_tx_clk : out STD_LOGIC;
        hdmi_tx_data : out STD_LOGIC_VECTOR(23 downto 0);
        hdmi_tx_de : out STD_LOGIC;
        hdmi_tx_hsync : out STD_LOGIC;
        hdmi_tx_vsync : out STD_LOGIC
    );
end video_output_interpolated;

architecture Behavioral of video_output_interpolated is

    -- Video timing constants
    constant H_TOTAL : integer := H_ACTIVE + H_FRONT_PORCH + H_SYNC_WIDTH + H_BACK_PORCH;
    constant V_TOTAL : integer := V_ACTIVE + V_FRONT_PORCH + V_SYNC_WIDTH + V_BACK_PORCH;
    
    -- Timing counters
    signal h_counter : unsigned(11 downto 0) := (others => '0');
    signal v_counter : unsigned(11 downto 0) := (others => '0');
    
    -- Interpolation coordinates
    signal interp_x : signed(15+INTERPOLATION_BITS downto 0);
    signal interp_y : signed(15+INTERPOLATION_BITS downto 0);
    
    -- Pixel coordinates for 2x2 interpolation
    signal pixel_x0, pixel_x1 : unsigned(11 downto 0);
    signal pixel_y0, pixel_y1 : unsigned(11 downto 0);
    
    -- Interpolation weights
    signal weight_x, weight_y : unsigned(INTERPOLATION_BITS-1 downto 0);
    
    -- Fetched pixel data
    signal pixel_00, pixel_01, pixel_10, pixel_11 : STD_LOGIC_VECTOR(23 downto 0);
    
    -- Interpolated result
    signal interpolated_pixel : STD_LOGIC_VECTOR(23 downto 0);
    
    -- Control signals
    signal video_active : STD_LOGIC;
    signal hsync_pulse : STD_LOGIC;
    signal vsync_pulse : STD_LOGIC;

begin

    hdmi_tx_clk <= pixel_clk;

    -- Video timing (similar to basic version)
    timing_gen: process(pixel_clk, rst)
    begin
        if rst = '1' then
            h_counter <= (others => '0');
            v_counter <= (others => '0');
        elsif rising_edge(pixel_clk) then
            if h_counter = H_TOTAL - 1 then
                h_counter <= (others => '0');
                if v_counter = V_TOTAL - 1 then
                    v_counter <= (others => '0');
                else
                    v_counter <= v_counter + 1;
                end if;
            else
                h_counter <= h_counter + 1;
            end if;
        end if;
    end process;

    -- Calculate interpolated coordinates
    interp_coords: process(pixel_clk)
    begin
        if rising_edge(pixel_clk) then
            if offset_valid = '1' then
                interp_x <= signed(resize(h_counter, 16)) * (2**INTERPOLATION_BITS) + signed(offset_x);
                interp_y <= signed(resize(v_counter, 16)) * (2**INTERPOLATION_BITS) + signed(offset_y);
            else
                interp_x <= signed(resize(h_counter, 16)) * (2**INTERPOLATION_BITS);
                interp_y <= signed(resize(v_counter, 16)) * (2**INTERPOLATION_BITS);
            end if;
        end if;
    end process;

    -- Extract pixel coordinates and weights
    pixel_x0 <= unsigned(interp_x(15+INTERPOLATION_BITS downto INTERPOLATION_BITS));
    pixel_y0 <= unsigned(interp_y(15+INTERPOLATION_BITS downto INTERPOLATION_BITS));
    pixel_x1 <= pixel_x0 + 1;
    pixel_y1 <= pixel_y0 + 1;
    
    weight_x <= unsigned(interp_x(INTERPOLATION_BITS-1 downto 0));
    weight_y <= unsigned(interp_y(INTERPOLATION_BITS-1 downto 0));

    -- Bilinear interpolation (simplified for one color component)
    bilinear_interp: process(pixel_clk)
        variable temp_00, temp_01, temp_10, temp_11 : unsigned(31 downto 0);
        variable interp_top, interp_bottom : unsigned(31 downto 0);
        variable final_result : unsigned(31 downto 0);
    begin
        if rising_edge(pixel_clk) then
            -- This is a simplified version - full implementation would
            -- need to handle all RGB components separately
            
            -- For now, pass through the nearest pixel
            interpolated_pixel <= frame_data;
        end if;
    end process;

    -- Generate control signals
    video_active <= '1' when (h_counter < H_ACTIVE and v_counter < V_ACTIVE) else '0';
    hsync_pulse <= '1' when (h_counter >= H_ACTIVE + H_FRONT_PORCH and 
                            h_counter < H_ACTIVE + H_FRONT_PORCH + H_SYNC_WIDTH) else '0';
    vsync_pulse <= '1' when (v_counter >= V_ACTIVE + V_FRONT_PORCH and 
                            v_counter < V_ACTIVE + V_FRONT_PORCH + V_SYNC_WIDTH) else '0';

    -- Output assignments
    hdmi_tx_data <= interpolated_pixel when video_active = '1' else x"000000";
    hdmi_tx_de <= video_active;
    hdmi_tx_hsync <= hsync_pulse;
    hdmi_tx_vsync <= vsync_pulse;

    -- Frame buffer request
    frame_addr <= std_logic_vector(resize(pixel_y0, 10) * H_ACTIVE + resize(pixel_x0, 10));
    frame_req <= video_active;

end Behavioral;
