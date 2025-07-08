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
