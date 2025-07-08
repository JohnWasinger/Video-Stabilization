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
