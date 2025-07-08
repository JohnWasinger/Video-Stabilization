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
