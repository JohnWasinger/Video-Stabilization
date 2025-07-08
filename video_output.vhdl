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
