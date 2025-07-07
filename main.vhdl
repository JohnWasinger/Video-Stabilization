-- HDMI Video Stabilization System
-- This system captures HDMI video, performs motion detection and compensation
-- to stabilize shaky video feeds

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Top-level entity for video stabilization
entity video_stabilizer is
    Port (
        clk_100mhz : in STD_LOGIC;
        rst : in STD_LOGIC;
        
        -- HDMI Input Interface
        hdmi_rx_clk : in STD_LOGIC;
        hdmi_rx_data : in STD_LOGIC_VECTOR(23 downto 0);
        hdmi_rx_de : in STD_LOGIC;
        hdmi_rx_hsync : in STD_LOGIC;
        hdmi_rx_vsync : in STD_LOGIC;
        
        -- HDMI Output Interface
        hdmi_tx_clk : out STD_LOGIC;
        hdmi_tx_data : out STD_LOGIC_VECTOR(23 downto 0);
        hdmi_tx_de : out STD_LOGIC;
        hdmi_tx_hsync : out STD_LOGIC;
        hdmi_tx_vsync : out STD_LOGIC;
        
        -- External Memory Interface (DDR3)
        ddr3_clk : out STD_LOGIC;
        ddr3_addr : out STD_LOGIC_VECTOR(13 downto 0);
        ddr3_ba : out STD_LOGIC_VECTOR(2 downto 0);
        ddr3_cas_n : out STD_LOGIC;
        ddr3_ras_n : out STD_LOGIC;
        ddr3_we_n : out STD_LOGIC;
        ddr3_dq : inout STD_LOGIC_VECTOR(31 downto 0);
        ddr3_dqs_p : inout STD_LOGIC_VECTOR(3 downto 0);
        ddr3_dqs_n : inout STD_LOGIC_VECTOR(3 downto 0);
        ddr3_dm : out STD_LOGIC_VECTOR(3 downto 0);
        ddr3_odt : out STD_LOGIC;
        ddr3_cke : out STD_LOGIC;
        ddr3_cs_n : out STD_LOGIC;
        ddr3_rst_n : out STD_LOGIC;
        
        -- Status and Control
        stabilization_enable : in STD_LOGIC;
        motion_threshold : in STD_LOGIC_VECTOR(15 downto 0);
        status_led : out STD_LOGIC_VECTOR(3 downto 0)
    );
end video_stabilizer;

architecture Behavioral of video_stabilizer is

    -- Component declarations
    component hdmi_receiver is
        Port (
            clk : in STD_LOGIC;
            rst : in STD_LOGIC;
            hdmi_clk : in STD_LOGIC;
            hdmi_data : in STD_LOGIC_VECTOR(23 downto 0);
            hdmi_de : in STD_LOGIC;
            hdmi_hsync : in STD_LOGIC;
            hdmi_vsync : in STD_LOGIC;
            pixel_clk : out STD_LOGIC;
            pixel_data : out STD_LOGIC_VECTOR(23 downto 0);
            pixel_valid : out STD_LOGIC;
            frame_start : out STD_LOGIC;
            line_start : out STD_LOGIC
        );
    end component;

    component motion_detector is
        Port (
            clk : in STD_LOGIC;
            rst : in STD_LOGIC;
            pixel_clk : in STD_LOGIC;
            current_frame : in STD_LOGIC_VECTOR(23 downto 0);
            previous_frame : in STD_LOGIC_VECTOR(23 downto 0);
            pixel_valid : in STD_LOGIC;
            frame_start : in STD_LOGIC;
            motion_threshold : in STD_LOGIC_VECTOR(15 downto 0);
            motion_vector_x : out STD_LOGIC_VECTOR(15 downto 0);
            motion_vector_y : out STD_LOGIC_VECTOR(15 downto 0);
            motion_valid : out STD_LOGIC
        );
    end component;

    component frame_buffer is
        Port (
            clk : in STD_LOGIC;
            rst : in STD_LOGIC;
            write_clk : in STD_LOGIC;
            write_addr : in STD_LOGIC_VECTOR(19 downto 0);
            write_data : in STD_LOGIC_VECTOR(23 downto 0);
            write_en : in STD_LOGIC;
            read_clk : in STD_LOGIC;
            read_addr : in STD_LOGIC_VECTOR(19 downto 0);
            read_data : out STD_LOGIC_VECTOR(23 downto 0);
            read_en : in STD_LOGIC
        );
    end component;

    component stabilization_processor is
        Port (
            clk : in STD_LOGIC;
            rst : in STD_LOGIC;
            enable : in STD_LOGIC;
            motion_vector_x : in STD_LOGIC_VECTOR(15 downto 0);
            motion_vector_y : in STD_LOGIC_VECTOR(15 downto 0);
            motion_valid : in STD_LOGIC;
            offset_x : out STD_LOGIC_VECTOR(15 downto 0);
            offset_y : out STD_LOGIC_VECTOR(15 downto 0);
            offset_valid : out STD_LOGIC
        );
    end component;

    component video_output is
        Port (
            clk : in STD_LOGIC;
            rst : in STD_LOGIC;
            pixel_clk : in STD_LOGIC;
            frame_data : in STD_LOGIC_VECTOR(23 downto 0);
            offset_x : in STD_LOGIC_VECTOR(15 downto 0);
            offset_y : in STD_LOGIC_VECTOR(15 downto 0);
            offset_valid : in STD_LOGIC;
            hdmi_tx_clk : out STD_LOGIC;
            hdmi_tx_data : out STD_LOGIC_VECTOR(23 downto 0);
            hdmi_tx_de : out STD_LOGIC;
            hdmi_tx_hsync : out STD_LOGIC;
            hdmi_tx_vsync : out STD_LOGIC
        );
    end component;

    -- Internal signals
    signal pixel_clk : STD_LOGIC;
    signal pixel_data : STD_LOGIC_VECTOR(23 downto 0);
    signal pixel_valid : STD_LOGIC;
    signal frame_start : STD_LOGIC;
    signal line_start : STD_LOGIC;
    
    signal current_frame_data : STD_LOGIC_VECTOR(23 downto 0);
    signal previous_frame_data : STD_LOGIC_VECTOR(23 downto 0);
    
    signal motion_vector_x : STD_LOGIC_VECTOR(15 downto 0);
    signal motion_vector_y : STD_LOGIC_VECTOR(15 downto 0);
    signal motion_valid : STD_LOGIC;
    
    signal offset_x : STD_LOGIC_VECTOR(15 downto 0);
    signal offset_y : STD_LOGIC_VECTOR(15 downto 0);
    signal offset_valid : STD_LOGIC;
    
    signal write_addr : STD_LOGIC_VECTOR(19 downto 0);
    signal read_addr : STD_LOGIC_VECTOR(19 downto 0);
    signal frame_write_en : STD_LOGIC;
    signal frame_read_en : STD_LOGIC;
    
    signal frame_counter : unsigned(1 downto 0);
    signal processing_active : STD_LOGIC;

begin

    -- Status LEDs
    status_led(0) <= processing_active;
    status_led(1) <= motion_valid;
    status_led(2) <= offset_valid;
    status_led(3) <= stabilization_enable;

    -- HDMI Receiver Instance
    hdmi_rx_inst: hdmi_receiver
        port map (
            clk => clk_100mhz,
            rst => rst,
            hdmi_clk => hdmi_rx_clk,
            hdmi_data => hdmi_rx_data,
            hdmi_de => hdmi_rx_de,
            hdmi_hsync => hdmi_rx_hsync,
            hdmi_vsync => hdmi_rx_vsync,
            pixel_clk => pixel_clk,
            pixel_data => pixel_data,
            pixel_valid => pixel_valid,
            frame_start => frame_start,
            line_start => line_start
        );

    -- Current Frame Buffer
    current_frame_buffer: frame_buffer
        port map (
            clk => clk_100mhz,
            rst => rst,
            write_clk => pixel_clk,
            write_addr => write_addr,
            write_data => pixel_data,
            write_en => frame_write_en,
            read_clk => pixel_clk,
            read_addr => read_addr,
            read_data => current_frame_data,
            read_en => frame_read_en
        );

    -- Previous Frame Buffer
    previous_frame_buffer: frame_buffer
        port map (
            clk => clk_100mhz,
            rst => rst,
            write_clk => pixel_clk,
            write_addr => write_addr,
            write_data => current_frame_data,
            write_en => frame_write_en,
            read_clk => pixel_clk,
            read_addr => read_addr,
            read_data => previous_frame_data,
            read_en => frame_read_en
        );

    -- Motion Detection
    motion_detect_inst: motion_detector
        port map (
            clk => clk_100mhz,
            rst => rst,
            pixel_clk => pixel_clk,
            current_frame => current_frame_data,
            previous_frame => previous_frame_data,
            pixel_valid => pixel_valid,
            frame_start => frame_start,
            motion_threshold => motion_threshold,
            motion_vector_x => motion_vector_x,
            motion_vector_y => motion_vector_y,
            motion_valid => motion_valid
        );

    -- Stabilization Processor
    stabilization_inst: stabilization_processor
        port map (
            clk => clk_100mhz,
            rst => rst,
            enable => stabilization_enable,
            motion_vector_x => motion_vector_x,
            motion_vector_y => motion_vector_y,
            motion_valid => motion_valid,
            offset_x => offset_x,
            offset_y => offset_y,
            offset_valid => offset_valid
        );

    -- Video Output
    video_output_inst: video_output
        port map (
            clk => clk_100mhz,
            rst => rst,
            pixel_clk => pixel_clk,
            frame_data => current_frame_data,
            offset_x => offset_x,
            offset_y => offset_y,
            offset_valid => offset_valid,
            hdmi_tx_clk => hdmi_tx_clk,
            hdmi_tx_data => hdmi_tx_data,
            hdmi_tx_de => hdmi_tx_de,
            hdmi_tx_hsync => hdmi_tx_hsync,
            hdmi_tx_vsync => hdmi_tx_vsync
        );

    -- Address generation for frame buffers
    addr_gen_proc: process(pixel_clk, rst)
        variable pixel_count : unsigned(19 downto 0);
    begin
        if rst = '1' then
            pixel_count := (others => '0');
            write_addr <= (others => '0');
            read_addr <= (others => '0');
            frame_write_en <= '0';
            frame_read_en <= '0';
            processing_active <= '0';
        elsif rising_edge(pixel_clk) then
            if frame_start = '1' then
                pixel_count := (others => '0');
                processing_active <= '1';
            elsif pixel_valid = '1' then
                pixel_count := pixel_count + 1;
            end if;
            
            write_addr <= std_logic_vector(pixel_count);
            read_addr <= std_logic_vector(pixel_count);
            frame_write_en <= pixel_valid;
            frame_read_en <= pixel_valid;
        end if;
    end process;

end Behavioral;

-- Motion Detection Component
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity motion_detector is
    Port (
        clk : in STD_LOGIC;
        rst : in STD_LOGIC;
        pixel_clk : in STD_LOGIC;
        current_frame : in STD_LOGIC_VECTOR(23 downto 0);
        previous_frame : in STD_LOGIC_VECTOR(23 downto 0);
        pixel_valid : in STD_LOGIC;
        frame_start : in STD_LOGIC;
        motion_threshold : in STD_LOGIC_VECTOR(15 downto 0);
        motion_vector_x : out STD_LOGIC_VECTOR(15 downto 0);
        motion_vector_y : out STD_LOGIC_VECTOR(15 downto 0);
        motion_valid : out STD_LOGIC
    );
end motion_detector;

architecture Behavioral of motion_detector is
    signal current_gray : unsigned(7 downto 0);
    signal previous_gray : unsigned(7 downto 0);
    signal diff : signed(8 downto 0);
    signal abs_diff : unsigned(7 downto 0);
    signal motion_sum_x : signed(31 downto 0);
    signal motion_sum_y : signed(31 downto 0);
    signal pixel_count : unsigned(19 downto 0);
    signal motion_detected : STD_LOGIC;
    
    signal block_x : unsigned(9 downto 0);
    signal block_y : unsigned(9 downto 0);
    
begin

    -- Convert RGB to grayscale (simplified)
    rgb_to_gray_proc: process(pixel_clk)
        variable r, g, b : unsigned(7 downto 0);
    begin
        if rising_edge(pixel_clk) then
            r := unsigned(current_frame(23 downto 16));
            g := unsigned(current_frame(15 downto 8));
            b := unsigned(current_frame(7 downto 0));
            current_gray <= (r + g + b) / 3;
            
            r := unsigned(previous_frame(23 downto 16));
            g := unsigned(previous_frame(15 downto 8));
            b := unsigned(previous_frame(7 downto 0));
            previous_gray <= (r + g + b) / 3;
        end if;
    end process;

    -- Motion detection using block matching
    motion_detect_proc: process(pixel_clk, rst)
        variable threshold : unsigned(15 downto 0);
    begin
        if rst = '1' then
            motion_sum_x <= (others => '0');
            motion_sum_y <= (others => '0');
            pixel_count <= (others => '0');
            motion_detected <= '0';
            motion_valid <= '0';
            block_x <= (others => '0');
            block_y <= (others => '0');
        elsif rising_edge(pixel_clk) then
            threshold := unsigned(motion_threshold);
            
            if frame_start = '1' then
                motion_sum_x <= (others => '0');
                motion_sum_y <= (others => '0');
                pixel_count <= (others => '0');
                motion_detected <= '0';
                motion_valid <= '0';
                block_x <= (others => '0');
                block_y <= (others => '0');
            elsif pixel_valid = '1' then
                -- Calculate difference
                diff <= signed('0' & current_gray) - signed('0' & previous_gray);
                
                if diff >= 0 then
                    abs_diff <= unsigned(diff(7 downto 0));
                else
                    abs_diff <= unsigned(-diff(7 downto 0));
                end if;
                
                -- Accumulate motion vectors if above threshold
                if abs_diff > threshold(7 downto 0) then
                    motion_sum_x <= motion_sum_x + signed(resize(block_x, 32));
                    motion_sum_y <= motion_sum_y + signed(resize(block_y, 32));
                    motion_detected <= '1';
                end if;
                
                pixel_count <= pixel_count + 1;
                
                -- Update block coordinates (simplified)
                if block_x = 1279 then  -- Assuming 1280x720
                    block_x <= (others => '0');
                    if block_y = 719 then
                        block_y <= (others => '0');
                        -- End of frame processing
                        if motion_detected = '1' then
                            motion_vector_x <= std_logic_vector(motion_sum_x(15 downto 0));
                            motion_vector_y <= std_logic_vector(motion_sum_y(15 downto 0));
                            motion_valid <= '1';
                        else
                            motion_vector_x <= (others => '0');
                            motion_vector_y <= (others => '0');
                            motion_valid <= '0';
                        end if;
                    else
                        block_y <= block_y + 1;
                    end if;
                else
                    block_x <= block_x + 1;
                end if;
            end if;
        end if;
    end process;

end Behavioral;

-- Stabilization Processor Component
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity stabilization_processor is
    Port (
        clk : in STD_LOGIC;
        rst : in STD_LOGIC;
        enable : in STD_LOGIC;
        motion_vector_x : in STD_LOGIC_VECTOR(15 downto 0);
        motion_vector_y : in STD_LOGIC_VECTOR(15 downto 0);
        motion_valid : in STD_LOGIC;
        offset_x : out STD_LOGIC_VECTOR(15 downto 0);
        offset_y : out STD_LOGIC_VECTOR(15 downto 0);
        offset_valid : out STD_LOGIC
    );
end stabilization_processor;

architecture Behavioral of stabilization_processor is
    signal accumulated_x : signed(31 downto 0);
    signal accumulated_y : signed(31 downto 0);
    signal filtered_x : signed(31 downto 0);
    signal filtered_y : signed(31 downto 0);
    signal alpha : unsigned(15 downto 0) := x"0CCC"; -- 0.8 in Q15 format
    
begin

    -- Low-pass filter for smooth stabilization
    stabilization_proc: process(clk, rst)
        variable motion_x : signed(31 downto 0);
        variable motion_y : signed(31 downto 0);
        variable temp_x : signed(63 downto 0);
        variable temp_y : signed(63 downto 0);
    begin
        if rst = '1' then
            accumulated_x <= (others => '0');
            accumulated_y <= (others => '0');
            filtered_x <= (others => '0');
            filtered_y <= (others => '0');
            offset_x <= (others => '0');
            offset_y <= (others => '0');
            offset_valid <= '0';
        elsif rising_edge(clk) then
            if enable = '1' and motion_valid = '1' then
                motion_x := signed(motion_vector_x) * 256; -- Scale for precision
                motion_y := signed(motion_vector_y) * 256;
                
                -- Accumulate motion
                accumulated_x <= accumulated_x + motion_x;
                accumulated_y <= accumulated_y + motion_y;
                
                -- Apply low-pass filter: filtered = alpha * accumulated + (1-alpha) * filtered
                temp_x := signed(alpha) * accumulated_x + (32768 - signed(alpha)) * filtered_x;
                temp_y := signed(alpha) * accumulated_y + (32768 - signed(alpha)) * filtered_y;
                
                filtered_x <= temp_x(47 downto 16); -- Scale back
                filtered_y <= temp_y(47 downto 16);
                
                -- Output compensation offset (negative of filtered motion)
                offset_x <= std_logic_vector(-filtered_x(15 downto 0));
                offset_y <= std_logic_vector(-filtered_y(15 downto 0));
                offset_valid <= '1';
            else
                offset_valid <= '0';
            end if;
        end if;
    end process;

end Behavioral;
