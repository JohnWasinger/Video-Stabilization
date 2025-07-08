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
