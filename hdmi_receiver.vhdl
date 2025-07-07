-- HDMI Receiver Implementation
-- This module decodes HDMI/DVI video signals and extracts pixel data
-- Supports common video formats with automatic format detection

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity hdmi_receiver is
    Generic (
        -- Video timing parameters
        H_ACTIVE_DEFAULT : integer := 1280;
        V_ACTIVE_DEFAULT : integer := 720;
        H_FRONT_PORCH : integer := 110;
        H_SYNC_WIDTH : integer := 40;
        H_BACK_PORCH : integer := 220;
        V_FRONT_PORCH : integer := 5;
        V_SYNC_WIDTH : integer := 5;
        V_BACK_PORCH : integer := 20
    );
    Port (
        clk : in STD_LOGIC;  -- System clock
        rst : in STD_LOGIC;  -- Reset
        
        -- HDMI Input Interface
        hdmi_clk : in STD_LOGIC;  -- HDMI pixel clock
        hdmi_data : in STD_LOGIC_VECTOR(23 downto 0);  -- RGB data
        hdmi_de : in STD_LOGIC;   -- Data enable
        hdmi_hsync : in STD_LOGIC; -- Horizontal sync
        hdmi_vsync : in STD_LOGIC; -- Vertical sync
        
        -- Output Interface
        pixel_clk : out STD_LOGIC;
        pixel_data : out STD_LOGIC_VECTOR(23 downto 0);
        pixel_valid : out STD_LOGIC;
        frame_start : out STD_LOGIC;
        line_start : out STD_LOGIC;
        
        -- Status and Control
        video_locked : out STD_LOGIC;
        h_resolution : out STD_LOGIC_VECTOR(11 downto 0);
        v_resolution : out STD_LOGIC_VECTOR(11 downto 0);
        frame_rate : out STD_LOGIC_VECTOR(7 downto 0);
        
        -- Debug outputs
        debug_state : out STD_LOGIC_VECTOR(3 downto 0)
    );
end hdmi_receiver;

architecture Behavioral of hdmi_receiver is

    -- State machine for HDMI reception
    type hdmi_state_type is (
        IDLE,
        SYNC_DETECT,
        TIMING_MEASURE,
        VIDEO_ACTIVE,
        FRAME_SYNC
    );
    
    signal current_state : hdmi_state_type := IDLE;
    signal next_state : hdmi_state_type;
    
    -- Clock domain crossing signals
    signal hdmi_clk_sync : STD_LOGIC;
    signal hdmi_data_sync : STD_LOGIC_VECTOR(23 downto 0);
    signal hdmi_de_sync : STD_LOGIC;
    signal hdmi_hsync_sync : STD_LOGIC;
    signal hdmi_vsync_sync : STD_LOGIC;
    
    -- Synchronizer registers
    signal hdmi_de_r : STD_LOGIC_VECTOR(2 downto 0) := (others => '0');
    signal hdmi_hsync_r : STD_LOGIC_VECTOR(2 downto 0) := (others => '0');
    signal hdmi_vsync_r : STD_LOGIC_VECTOR(2 downto 0) := (others => '0');
    signal hdmi_data_r : STD_LOGIC_VECTOR(23 downto 0) := (others => '0');
    
    -- Edge detection
    signal hsync_rising : STD_LOGIC;
    signal hsync_falling : STD_LOGIC;
    signal vsync_rising : STD_LOGIC;
    signal vsync_falling : STD_LOGIC;
    signal de_rising : STD_LOGIC;
    signal de_falling : STD_LOGIC;
    
    -- Timing measurement
    signal h_counter : unsigned(11 downto 0) := (others => '0');
    signal v_counter : unsigned(11 downto 0) := (others => '0');
    signal h_active_count : unsigned(11 downto 0) := (others => '0');
    signal v_active_count : unsigned(11 downto 0) := (others => '0');
    signal h_total_count : unsigned(11 downto 0) := (others => '0');
    signal v_total_count : unsigned(11 downto 0) := (others => '0');
    
    -- Video timing parameters
    signal h_active : unsigned(11 downto 0) := to_unsigned(H_ACTIVE_DEFAULT, 12);
    signal v_active : unsigned(11 downto 0) := to_unsigned(V_ACTIVE_DEFAULT, 12);
    signal h_total : unsigned(11 downto 0) := to_unsigned(H_ACTIVE_DEFAULT + H_FRONT_PORCH + H_SYNC_WIDTH + H_BACK_PORCH, 12);
    signal v_total : unsigned(11 downto 0) := to_unsigned(V_ACTIVE_DEFAULT + V_FRONT_PORCH + V_SYNC_WIDTH + V_BACK_PORCH, 12);
    
    -- Frame rate measurement
    signal frame_counter : unsigned(15 downto 0) := (others => '0');
    signal frame_rate_counter : unsigned(23 downto 0) := (others => '0');
    signal frame_rate_reg : unsigned(7 downto 0) := (others => '0');
    
    -- Lock detection
    signal sync_lock_counter : unsigned(7 downto 0) := (others => '0');
    signal video_locked_reg : STD_LOGIC := '0';
    
    -- Output registers
    signal pixel_data_reg : STD_LOGIC_VECTOR(23 downto 0) := (others => '0');
    signal pixel_valid_reg : STD_LOGIC := '0';
    signal frame_start_reg : STD_LOGIC := '0';
    signal line_start_reg : STD_LOGIC := '0';
    
    -- Constants
    constant LOCK_THRESHOLD : unsigned(7 downto 0) := x"10"; -- 16 consecutive good frames
    constant FRAME_RATE_WINDOW : unsigned(23 downto 0) := x"5F5E10"; -- ~100ms at 100MHz

begin

    -- Output assignments
    pixel_clk <= hdmi_clk;
    pixel_data <= pixel_data_reg;
    pixel_valid <= pixel_valid_reg;
    frame_start <= frame_start_reg;
    line_start <= line_start_reg;
    video_locked <= video_locked_reg;
    h_resolution <= std_logic_vector(h_active);
    v_resolution <= std_logic_vector(v_active);
    frame_rate <= std_logic_vector(frame_rate_reg);
    
    -- Debug state output
    debug_state <= x"0" when current_state = IDLE else
                   x"1" when current_state = SYNC_DETECT else
                   x"2" when current_state = TIMING_MEASURE else
                   x"3" when current_state = VIDEO_ACTIVE else
                   x"4" when current_state = FRAME_SYNC else
                   x"F";

    -- Input synchronization process
    sync_inputs: process(hdmi_clk, rst)
    begin
        if rst = '1' then
            hdmi_de_r <= (others => '0');
            hdmi_hsync_r <= (others => '0');
            hdmi_vsync_r <= (others => '0');
            hdmi_data_r <= (others => '0');
        elsif rising_edge(hdmi_clk) then
            -- Synchronize inputs to HDMI clock domain
            hdmi_de_r <= hdmi_de_r(1 downto 0) & hdmi_de;
            hdmi_hsync_r <= hdmi_hsync_r(1 downto 0) & hdmi_hsync;
            hdmi_vsync_r <= hdmi_vsync_r(1 downto 0) & hdmi_vsync;
            hdmi_data_r <= hdmi_data;
        end if;
    end process;
    
    -- Extract synchronized signals
    hdmi_de_sync <= hdmi_de_r(2);
    hdmi_hsync_sync <= hdmi_hsync_r(2);
    hdmi_vsync_sync <= hdmi_vsync_r(2);
    hdmi_data_sync <= hdmi_data_r;
    
    -- Edge detection
    hsync_rising <= hdmi_hsync_r(2) and not hdmi_hsync_r(1);
    hsync_falling <= not hdmi_hsync_r(2) and hdmi_hsync_r(1);
    vsync_rising <= hdmi_vsync_r(2) and not hdmi_vsync_r(1);
    vsync_falling <= not hdmi_vsync_r(2) and hdmi_vsync_r(1);
    de_rising <= hdmi_de_r(2) and not hdmi_de_r(1);
    de_falling <= not hdmi_de_r(2) and hdmi_de_r(1);

    -- Main state machine
    state_machine: process(hdmi_clk, rst)
    begin
        if rst = '1' then
            current_state <= IDLE;
            h_counter <= (others => '0');
            v_counter <= (others => '0');
            h_active_count <= (others => '0');
            v_active_count <= (others => '0');
            h_total_count <= (others => '0');
            v_total_count <= (others => '0');
            sync_lock_counter <= (others => '0');
            video_locked_reg <= '0';
            pixel_data_reg <= (others => '0');
            pixel_valid_reg <= '0';
            frame_start_reg <= '0';
            line_start_reg <= '0';
            
        elsif rising_edge(hdmi_clk) then
            
            -- Default assignments
            frame_start_reg <= '0';
            line_start_reg <= '0';
            pixel_valid_reg <= '0';
            
            case current_state is
                
                when IDLE =>
                    -- Wait for stable sync signals
                    if hsync_rising = '1' or hsync_falling = '1' then
                        current_state <= SYNC_DETECT;
                        h_counter <= (others => '0');
                        v_counter <= (others => '0');
                    end if;
                    
                when SYNC_DETECT =>
                    -- Look for consistent sync pattern
                    h_counter <= h_counter + 1;
                    
                    if hsync_rising = '1' then
                        h_total_count <= h_counter;
                        h_counter <= (others => '0');
                        v_counter <= v_counter + 1;
                    end if;
                    
                    if vsync_rising = '1' then
                        v_total_count <= v_counter;
                        v_counter <= (others => '0');
                        current_state <= TIMING_MEASURE;
                    end if;
                    
                when TIMING_MEASURE =>
                    -- Measure active video timing
                    h_counter <= h_counter + 1;
                    
                    if de_rising = '1' then
                        line_start_reg <= '1';
                        h_active_count <= (others => '0');
                    elsif hdmi_de_sync = '1' then
                        h_active_count <= h_active_count + 1;
                    end if;
                    
                    if hsync_rising = '1' then
                        h_counter <= (others => '0');
                        v_counter <= v_counter + 1;
                        
                        if hdmi_de_sync = '1' then
                            v_active_count <= v_active_count + 1;
                        end if;
                    end if;
                    
                    if vsync_rising = '1' then
                        -- Store measured timing
                        h_active <= h_active_count;
                        v_active <= v_active_count;
                        h_total <= h_total_count;
                        v_total <= v_total_count;
                        
                        -- Reset counters
                        h_counter <= (others => '0');
                        v_counter <= (others => '0');
                        h_active_count <= (others => '0');
                        v_active_count <= (others => '0');
                        
                        current_state <= VIDEO_ACTIVE;
                    end if;
                    
                when VIDEO_ACTIVE =>
                    -- Normal video processing
                    h_counter <= h_counter + 1;
                    
                    -- Output pixel data when DE is active
                    if hdmi_de_sync = '1' then
                        pixel_data_reg <= hdmi_data_sync;
                        pixel_valid_reg <= '1';
                    end if;
                    
                    -- Line start detection
                    if de_rising = '1' then
                        line_start_reg <= '1';
                    end if;
                    
                    -- Frame start detection
                    if vsync_rising = '1' then
                        frame_start_reg <= '1';
                        h_counter <= (others => '0');
                        v_counter <= (others => '0');
                        
                        -- Check for stable timing
                        if sync_lock_counter < LOCK_THRESHOLD then
                            sync_lock_counter <= sync_lock_counter + 1;
                        else
                            video_locked_reg <= '1';
                        end if;
                    end if;
                    
                    -- Line end detection
                    if hsync_rising = '1' then
                        h_counter <= (others => '0');
                        v_counter <= v_counter + 1;
                    end if;
                    
                    -- Check for timing violations
                    if h_counter > h_total + 10 or v_counter > v_total + 10 then
                        current_state <= SYNC_DETECT;
                        video_locked_reg <= '0';
                        sync_lock_counter <= (others => '0');
                    end if;
                    
                when FRAME_SYNC =>
                    -- Resynchronize if needed
                    if vsync_rising = '1' then
                        current_state <= VIDEO_ACTIVE;
                    end if;
                    
                when others =>
                    current_state <= IDLE;
                    
            end case;
        end if;
    end process;

    -- Frame rate measurement
    frame_rate_measure: process(clk, rst)
    begin
        if rst = '1' then
            frame_counter <= (others => '0');
            frame_rate_counter <= (others => '0');
            frame_rate_reg <= (others => '0');
        elsif rising_edge(clk) then
            frame_rate_counter <= frame_rate_counter + 1;
            
            -- Count frames from HDMI clock domain (simplified)
            if frame_start_reg = '1' then
                frame_counter <= frame_counter + 1;
            end if;
            
            -- Update frame rate every window period
            if frame_rate_counter = FRAME_RATE_WINDOW then
                frame_rate_reg <= frame_counter(7 downto 0);
                frame_counter <= (others => '0');
                frame_rate_counter <= (others => '0');
            end if;
        end if;
    end process;

end Behavioral;

-- HDMI Clock Recovery and Data Alignment (Optional Enhancement)
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity hdmi_clock_recovery is
    Port (
        ref_clk : in STD_LOGIC;  -- Reference clock (e.g., 100MHz)
        rst : in STD_LOGIC;
        
        -- HDMI differential inputs (if using TMDS)
        hdmi_clk_p : in STD_LOGIC;
        hdmi_clk_n : in STD_LOGIC;
        hdmi_data_p : in STD_LOGIC_VECTOR(2 downto 0);
        hdmi_data_n : in STD_LOGIC_VECTOR(2 downto 0);
        
        -- Recovered clock and data
        recovered_clk : out STD_LOGIC;
        recovered_data : out STD_LOGIC_VECTOR(23 downto 0);
        recovered_de : out STD_LOGIC;
        recovered_hsync : out STD_LOGIC;
        recovered_vsync : out STD_LOGIC;
        
        -- Status
        clock_locked : out STD_LOGIC;
        data_valid : out STD_LOGIC
    );
end hdmi_clock_recovery;

architecture Behavioral of hdmi_clock_recovery is

    -- TMDS decoder component (vendor-specific)
    component tmds_decoder is
        Port (
            clk : in STD_LOGIC;
            rst : in STD_LOGIC;
            tmds_data : in STD_LOGIC_VECTOR(9 downto 0);
            decoded_data : out STD_LOGIC_VECTOR(7 downto 0);
            is_control : out STD_LOGIC;
            is_data : out STD_LOGIC;
            control_code : out STD_LOGIC_VECTOR(1 downto 0)
        );
    end component;
    
    -- Internal signals
    signal hdmi_clk_buf : STD_LOGIC;
    signal hdmi_data_buf : STD_LOGIC_VECTOR(2 downto 0);
    signal tmds_clk : STD_LOGIC;
    signal tmds_data : STD_LOGIC_VECTOR(29 downto 0);
    
    -- Deserialized data
    signal channel_0_data : STD_LOGIC_VECTOR(9 downto 0);
    signal channel_1_data : STD_LOGIC_VECTOR(9 downto 0);
    signal channel_2_data : STD_LOGIC_VECTOR(9 downto 0);
    
    -- Decoded outputs
    signal decoded_blue : STD_LOGIC_VECTOR(7 downto 0);
    signal decoded_green : STD_LOGIC_VECTOR(7 downto 0);
    signal decoded_red : STD_LOGIC_VECTOR(7 downto 0);
    
    signal control_valid : STD_LOGIC_VECTOR(2 downto 0);
    signal data_valid_ch : STD_LOGIC_VECTOR(2 downto 0);
    signal control_codes : STD_LOGIC_VECTOR(5 downto 0);
    
    -- Clock management
    signal pll_locked : STD_LOGIC;
    signal pixel_clk : STD_LOGIC;
    signal serial_clk : STD_LOGIC;

begin

    -- Differential input buffers (platform-specific)
    -- This would typically use IBUFDS for Xilinx or similar for other vendors
    hdmi_clk_buf <= hdmi_clk_p; -- Simplified - use proper differential buffer
    hdmi_data_buf(0) <= hdmi_data_p(0); -- Blue
    hdmi_data_buf(1) <= hdmi_data_p(1); -- Green  
    hdmi_data_buf(2) <= hdmi_data_p(2); -- Red
    
    -- Clock recovery PLL (platform-specific)
    -- This would typically use MMCM/PLL primitives
    tmds_clk <= hdmi_clk_buf;  -- Simplified
    pixel_clk <= tmds_clk;     -- Typically divided by 10
    serial_clk <= tmds_clk;    -- Typically multiplied by 5
    
    recovered_clk <= pixel_clk;
    clock_locked <= pll_locked;
    
    -- TMDS deserializers would go here (platform-specific)
    -- For now, assume we have the 10-bit parallel data
    
    -- Channel 0 (Blue + Control)
    tmds_decode_ch0: tmds_decoder
        port map (
            clk => pixel_clk,
            rst => rst,
            tmds_data => channel_0_data,
            decoded_data => decoded_blue,
            is_control => control_valid(0),
            is_data => data_valid_ch(0),
            control_code => control_codes(1 downto 0)
        );
    
    -- Channel 1 (Green)
    tmds_decode_ch1: tmds_decoder
        port map (
            clk => pixel_clk,
            rst => rst,
            tmds_data => channel_1_data,
            decoded_data => decoded_green,
            is_control => control_valid(1),
            is_data => data_valid_ch(1),
            control_code => open
        );
    
    -- Channel 2 (Red)
    tmds_decode_ch2: tmds_decoder
        port map (
            clk => pixel_clk,
            rst => rst,
            tmds_data => channel_2_data,
            decoded_data => decoded_red,
            is_control => control_valid(2),
            is_data => data_valid_ch(2),
            control_code => open
        );
    
    -- Extract control signals from channel 0
    recovered_hsync <= control_codes(0) when control_valid(0) = '1' else '0';
    recovered_vsync <= control_codes(1) when control_valid(0) = '1' else '0';
    recovered_de <= data_valid_ch(0) and data_valid_ch(1) and data_valid_ch(2);
    
    -- Combine RGB data
    recovered_data <= decoded_red & decoded_green & decoded_blue;
    data_valid <= recovered_de;

end Behavioral;
