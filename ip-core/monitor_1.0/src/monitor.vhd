-----------------------------------------------------------------------------
-- Monitor Infrastucture                                                   --
--                                                                         --
-- Author: Juan Encinas <juan.encinas@upm.es>                              --
--                                                                         --
-- NOTE: - Traces have one clock cycle of delay                            --
--       - After done = '1' a stop = '1' is needed to move back to IDLE    --
--       - CS_n negative polatity and SCLK positive polarity are assumed   --
--       - The SPI clock frequency will be the closest_below posible freq  --
--         to achievable with the CLK_FREQ                                 --
--       - SCLK_FREQ should be < CLK_FREQ/2,                               --
--         otherwise SCLK_FREQ = CLK_FREQ/2                                --
-----------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity monitor is
    generic (
        CLK_FREQ               : integer := 100;        -- Clock frequency in Hz
        SCLK_FREQ              : integer := 20;         -- SPI Clock frequency in Hz
        ADC_ENABLE             : boolean := true;       -- Power consumption monitoring enable (true, false)
        ADC_DUAL               : boolean := true;       -- Indicate if the ADC uses two channels (1 and 3) [true] or just one (0-USB) [false]
        ADC_VREF_IS_DOUBLE     : boolean := false;      -- Indicate the ADC voltage reference (false: 2.5V, true: 5.0V)
        COUNTER_BITS           : integer := 32;         -- Number of bits used for the counter count
        NUMBER_PROBES          : integer := 32;         -- Number of accelerator-related digital probes
        AXI_SNIFFER_ENABLE     : boolean := false;       -- AXI BUS SNIFFER ENABLE (ENABLED, DISABLED)
        AXI_SNIFFER_DATA_WIDTH : integer := 0;          -- AXI bus data width to be sniffed
        POWER_DATA_WIDTH       : integer := 32;         -- Power AXI bus data width
        POWER_ADDR_WIDTH       : integer := 32;         -- Power AXI bus addresss width
        POWER_DEPTH            : integer := 4096;       -- Number of power measurement to store (maximum)
        TRACES_DATA_WIDTH      : integer := 64;         -- Traces AXI bus data width
        TRACES_ADDR_WIDTH      : integer := 32;         -- Traces AXI bus addresss width
        TRACES_DEPTH           : integer := 4096        -- Number of traces samples to store (maximum)
    );
    port (
        -- Clock and reset signals
        clk                   : in std_logic;
        rst_n                 : in std_logic;
        -- Configuration signals
        user_config_vref      : in std_logic_vector(1 downto 0);
        -- Start and stop signal
        start                 : in std_logic;
        stop                  : in std_logic;
        -- HW accelerator probes
        probes                : in std_logic_vector(AXI_SNIFFER_DATA_WIDTH + NUMBER_PROBES - 1 downto 0);
        probes_mask           : in std_logic_vector(NUMBER_PROBES-1 downto 0);
        axi_sniffer_en        : in std_logic;
        axi_sniffer_mask      : in std_logic_vector(AXI_SNIFFER_DATA_WIDTH-1 downto 0);
        -- Busy and done signals
        busy                  : out std_logic;
        done                  : out std_logic;
        -- Output counter count used to register elapsed time in clock cycles
        count                 : out std_logic_vector(COUNTER_BITS-1 downto 0);
        -- Errors counter count used to tell the user how many power measurements were wrong
        power_errors          : out std_logic_vector(POWER_ADDR_WIDTH-1 downto 0);

        ---------------------------------------
        -- External reading of BRAMs signals --
        ---------------------------------------

        -- Enable
        power_bram_read_en    : in std_logic;
        traces_bram_read_en   : in std_logic;

        -- Address
        power_bram_read_addr  : in std_logic_vector(POWER_ADDR_WIDTH-1 downto 0);
        traces_bram_read_addr : in std_logic_vector(TRACES_ADDR_WIDTH-1 downto 0);

        -- Output
        power_bram_read_dout  : out std_logic_vector(POWER_DATA_WIDTH-1 downto 0);
        traces_bram_read_dout : out std_logic_vector(TRACES_DATA_WIDTH-1 downto 0);

        -- Utilization (last address written)
        power_bram_utilization  : out std_logic_vector(POWER_ADDR_WIDTH-1 downto 0);
        traces_bram_utilization : out std_logic_vector(TRACES_ADDR_WIDTH-1 downto 0);

        ---------
        -- SPI --
        ---------

        -- Chip select (negative polatiry)
        SPI_CS_n : out STD_LOGIC;
        -- Clock
        SPI_SCLK : out STD_LOGIC;
        -- Master In - Slave Out
        SPI_MISO : in STD_LOGIC;
        -- Master Out - Slave In
        SPI_MOSI : out STD_LOGIC
    );
    -- DEBUG
    attribute mark_debug : string;
    attribute mark_debug of user_config_vref        : signal is "TRUE";
    attribute mark_debug of start                   : signal is "TRUE";
    attribute mark_debug of stop                    : signal is "TRUE";
    attribute mark_debug of probes                  : signal is "TRUE";
    attribute mark_debug of probes_mask             : signal is "TRUE";
    attribute mark_debug of axi_sniffer_en          : signal is "TRUE";
    attribute mark_debug of axi_sniffer_mask        : signal is "TRUE";
    attribute mark_debug of busy                    : signal is "TRUE";
    attribute mark_debug of done                    : signal is "TRUE";
    attribute mark_debug of count                   : signal is "TRUE";
    attribute mark_debug of power_errors            : signal is "TRUE";
    attribute mark_debug of power_bram_read_en      : signal is "TRUE";
    attribute mark_debug of traces_bram_read_en     : signal is "TRUE";
    attribute mark_debug of power_bram_read_addr    : signal is "TRUE";
    attribute mark_debug of traces_bram_read_addr   : signal is "TRUE";
    attribute mark_debug of power_bram_read_dout    : signal is "TRUE";
    attribute mark_debug of traces_bram_read_dout   : signal is "TRUE";
    attribute mark_debug of power_bram_utilization  : signal is "TRUE";
    attribute mark_debug of traces_bram_utilization : signal is "TRUE";
    attribute mark_debug of SPI_CS_n                : signal is "TRUE";
    attribute mark_debug of SPI_SCLK                : signal is "TRUE";
    attribute mark_debug of SPI_MISO                : signal is "TRUE";
    attribute mark_debug of SPI_MOSI                : signal is "TRUE";
end monitor;

architecture Behavioral of monitor is

    -- Power error constant (when the power errors counter reach this value the monitor goes to done)
    constant POWER_MEASUREMENT_FAILURE: std_logic_vector(POWER_ADDR_WIDTH-1 downto 0) := (others => '1');

    ----------------------
    -- Internal Signals --
    ----------------------

    -- General FSM (ADC_BUSY, IDLE, USER_ADC_CONFIGURATION, CAPTURE, READ)
    type state_t is (S_ADC_BUSY, S_IDLE, S_CONFIGURATION, S_CAPTURE, S_READ, S_CLEAR);
    signal state : state_t;

    -- ADC Capture steps FSM (ADC_START, ADC_WAIT)
    type capture_step_t is (S_ADC_START, S_ADC_WAIT);
    signal capture_step : capture_step_t;

    -- ADC Clear steps FSM (WAIT_READY, START_CLEAR, GO_BUSY)
    type clear_step_t is (S_WAIT_READY, S_START_CLEAR, S_GO_BUSY);
    signal clear_step : clear_step_t;


    -- Probes Trigger Module (ptm)
    signal ptm_trigger : std_logic;

    -- AXI Trigger Module (atm)
    signal atm_trigger : std_logic :='0';

    -- Events Detector
    signal probes_delayed : std_logic_vector(probes'range);
    signal event_detected : std_logic;
    signal edges          : std_logic_vector(probes'range);

    -- ADC Manager (adc)
    signal adc_ready       : std_logic;
    signal power_data      : std_logic_vector(15 downto 0);

    -- Timestamp Counter
    signal counter_count : std_logic_vector(COUNTER_BITS-1 downto 0);

    -- Error Counter
    signal power_errors_count  : std_logic_vector(POWER_ADDR_WIDTH-1 downto 0);
    signal power_error_s       : std_logic;

    -- BRAM management signals
    signal power_bram_we     : std_logic;
    signal power_bram_we_fix : std_logic;
    signal power_bram_addr   : unsigned(POWER_ADDR_WIDTH-1 downto 0);
    signal power_bram_full   : std_logic;

    signal traces_bram_we   : std_logic;
    signal traces_bram_addr : unsigned(TRACES_ADDR_WIDTH-1 downto 0);
    signal traces_bram_full : std_logic;

    -- Local
    signal user_config_vref_reg : std_logic_vector(1 downto 0);
    signal internal_start       : std_logic;
    signal trigger_enable       : std_logic;
    signal acd_enable           : std_logic;
    signal traces_enable        : std_logic;
    signal initial_conditions   : std_logic;
    signal probes_bram_data     : std_logic_vector(probes'range);
    signal clear                : std_logic;

    -- DEBUG
    attribute mark_debug of state                : signal is "TRUE";
    attribute mark_debug of capture_step         : signal is "TRUE";
    attribute mark_debug of clear_step           : signal is "TRUE";
    attribute mark_debug of ptm_trigger          : signal is "TRUE";
    attribute mark_debug of atm_trigger          : signal is "TRUE";
    attribute mark_debug of probes_delayed       : signal is "TRUE";
    attribute mark_debug of event_detected       : signal is "TRUE";
    attribute mark_debug of edges                : signal is "TRUE";
    attribute mark_debug of adc_ready            : signal is "TRUE";
    attribute mark_debug of power_data           : signal is "TRUE";
    attribute mark_debug of counter_count        : signal is "TRUE";
    attribute mark_debug of power_errors_count   : signal is "TRUE";
    attribute mark_debug of power_bram_we        : signal is "TRUE";
    attribute mark_debug of power_bram_we_fix    : signal is "TRUE";
    attribute mark_debug of power_bram_addr      : signal is "TRUE";
    attribute mark_debug of power_bram_full      : signal is "TRUE";
    attribute mark_debug of traces_bram_we       : signal is "TRUE";
    attribute mark_debug of traces_bram_addr     : signal is "TRUE";
    attribute mark_debug of traces_bram_full     : signal is "TRUE";
    attribute mark_debug of user_config_vref_reg : signal is "TRUE";
    attribute mark_debug of internal_start       : signal is "TRUE";
    attribute mark_debug of trigger_enable       : signal is "TRUE";
    attribute mark_debug of acd_enable           : signal is "TRUE";
    attribute mark_debug of traces_enable        : signal is "TRUE";
    attribute mark_debug of initial_conditions   : signal is "TRUE";
    attribute mark_debug of probes_bram_data     : signal is "TRUE";
    attribute mark_debug of clear                : signal is "TRUE";

begin

    -----------------------------
    -- Submodules Instatiation --
    -----------------------------

    -- Instantiation of the Probes Trigger Module
    probes_trigger_module: entity work.probes_trigger_module
        generic map (NUMBER_INPUTS => NUMBER_PROBES)
        port map (
            clk     => clk,
            rst_n   => rst_n,
            en      => trigger_enable,
            inputs  => probes(NUMBER_PROBES+AXI_SNIFFER_DATA_WIDTH-1 downto AXI_SNIFFER_DATA_WIDTH),
            mask    => probes_mask,
            trigger => ptm_trigger
        );

    -- Instantiation of the AXI Trigger Module (with generate, conditional)
    axi_sniffing_enabler: if AXI_SNIFFER_ENABLE = true generate
    signal axi_trigger_enable : std_logic;
    begin

        axi_trigger_enable <= '1' when trigger_enable = '1' and axi_sniffer_en = '1' else
                              '0';

        axi_trigger_module: entity work.axi_trigger_module
            generic map (NUMBER_INPUTS => AXI_SNIFFER_DATA_WIDTH)
            port map (
                clk     => clk,
                rst_n   => rst_n,
                en      => axi_trigger_enable,
                inputs  => probes(AXI_SNIFFER_DATA_WIDTH-1 downto 0),
                mask    => axi_sniffer_mask,
                trigger => atm_trigger
            );

    end generate;

    -- Instantiation of the Events Detector
    events_detector: entity work.events_detector
        generic map (NUMBER_INPUTS => NUMBER_PROBES + AXI_SNIFFER_DATA_WIDTH)
        port map (
            clk            => clk,
            rst_n          => rst_n,
            en             => traces_enable,
            inputs         => probes,
            inputs_delayed => probes_delayed,
            event_detected => event_detected,
            edges          => edges
        );

    -- Instantiation of the Timestamp Counter
    timestamp_counter: entity work.counter
        generic map (BITS => COUNTER_BITS)
        port map (
            clk   => clk,
            rst_n => rst_n,
            en    => traces_enable,
            clr   => clear,
            count => counter_count
        );

    -- Instantiation of the Timestamps BRAM
    timestamps_bram: entity work.bram_dualport
        generic map (
            C_DATA_WIDTH => COUNTER_BITS,
            C_ADDR_WIDTH => TRACES_ADDR_WIDTH,
            C_MEM_DEPTH  => TRACES_DEPTH
        )
        port map (
            clk_a  => clk,
            en_a   => traces_bram_we,
            we_a   => traces_bram_we,
            addr_a => std_logic_vector(traces_bram_addr),
            din_a  => counter_count,
            dout_a => open,
            clk_b  => clk,
            en_b   => traces_bram_read_en,
            we_b   => '0',
            addr_b => traces_bram_read_addr,
            din_b  => (counter_count'range => '0'),
            dout_b => traces_bram_read_dout(COUNTER_BITS-1 downto 0)
            );

    -- Instantiation of the Probes BRAM
    probes_bram: entity work.bram_dualport
        generic map (
            C_DATA_WIDTH => NUMBER_PROBES + AXI_SNIFFER_DATA_WIDTH,
            C_ADDR_WIDTH => TRACES_ADDR_WIDTH,
            C_MEM_DEPTH  => TRACES_DEPTH
        )
        port map (
            clk_a  => clk,
            en_a   => traces_bram_we,
            we_a   => traces_bram_we,
            addr_a => std_logic_vector(traces_bram_addr),
            din_a  => probes_bram_data,
            dout_a => open,
            clk_b  => clk,
            en_b   => traces_bram_read_en,
            we_b   => '0',
            addr_b => traces_bram_read_addr,
            din_b  => (probes_bram_data'range => '0'),
            dout_b => traces_bram_read_dout((NUMBER_PROBES + AXI_SNIFFER_DATA_WIDTH + TRACES_DATA_WIDTH/2)-1 downto TRACES_DATA_WIDTH/2)
        );

    power_modules_enabler: if ADC_ENABLE = true generate

        -- Instantiation of the ADC Manager
        adc_manager: entity work.adc_manager
            generic map (
                CLK_FREQ           => CLK_FREQ,
                SCLK_FREQ          => SCLK_FREQ,
                ADC_DUAL           => ADC_DUAL,
                ADC_VREF_IS_DOUBLE => ADC_VREF_IS_DOUBLE
            )
            port map (
                clk         => clk,
                rst_n       => rst_n,
                config_vref => user_config_vref_reg,
                en          => acd_enable,
                clr         => clear,
                ready       => adc_ready,
                recv_data   => power_data,
                SPI_CS_n    => SPI_CS_n,
                SPI_SCLK    => SPI_SCLK,
                SPI_MISO    => SPI_MISO,
                SPI_MOSI    => SPI_MOSI
            );

        -- Instantiation of the Errors Counter
        errors_counter: entity work.counter
            generic map (BITS => POWER_ADDR_WIDTH)
            port map (
                clk   => clk,
                rst_n => rst_n,
                en    => power_error_s,
                clr   => clear,
                count => power_errors_count
            );

        -- Instantiation of the Power BRAM
        power_bram: entity work.bram_dualport
            generic map (
                C_DATA_WIDTH => 12,
                C_ADDR_WIDTH => POWER_ADDR_WIDTH,
                C_MEM_DEPTH  => POWER_DEPTH
            )
            port map (
                clk_a  => clk,
                en_a   => power_bram_we_fix,
                we_a   => power_bram_we_fix,
                addr_a => std_logic_vector(power_bram_addr),
                din_a  => power_data(11 downto 0),
                dout_a => open,
                clk_b  => clk,
                en_b   => power_bram_read_en,
                we_b   => '0',
                addr_b => power_bram_read_addr,
                din_b  => "000000000000",
                dout_b => power_bram_read_dout(11 downto 0)
            );
    end generate;

    -----------------
    -- Local Logic --
    -----------------

    -- Trigger (HIGH when the user signals a start or there is a probe or axi triggering) (configuration is prioritary)
    internal_start <= '1' when (start = '1' or ptm_trigger = '1' or atm_trigger = '1') and user_config_vref = "00" else '0';

    -- General FSM with power monitoring
    power_fsm_enabled: if ADC_ENABLE = true generate

        fsm: process(clk, rst_n)
        begin

            -- Asynchronous reset
            if rst_n = '0' then
                -- Reset states
                state                <= S_ADC_BUSY;
                capture_step         <= S_ADC_START;
                clear_step           <= S_WAIT_READY;
                -- Reset user configuration register
                user_config_vref_reg <= "00";
                initial_conditions   <= '0';
            -- Synchronous process
            elsif clk'event and clk = '1' then
                case state is

                    -- When the ADC is busy (SPI communication or ADC configuration)
                    when S_ADC_BUSY =>
                        -- Back to idle when ADC is ready
                        if adc_ready = '1' then
                            state <= S_IDLE;
                        end if;

                    -- ADC is ready
                    when S_IDLE =>
                        -- The user configuration has priority
                        if user_config_vref /= "00" then
                            -- Move to configuration state and register the user configuration value
                            state                <= S_CONFIGURATION;
                            user_config_vref_reg <= user_config_vref;
                        -- Start capturing when the is a trigger or start
                        elsif internal_start = '1' then
                            -- Move to Capture state and indicate that initial conditions (probes) must be captured
                            state              <= S_CAPTURE;
                            initial_conditions <= '1';
                        end if;

                    -- ADC configuration mode
                    when S_CONFIGURATION =>
                        -- Move to ADC busy state and clean the user config register
                        state                <= S_ADC_BUSY;
                        user_config_vref_reg <= "00";

                    -- Capture state (each of its steps are described by another FSM)
                    when S_CAPTURE =>
                        -- Clean initial conditions (just a pulse)
                        initial_conditions <= '0';
                        -- Keep capturing until the power or traces brams are full
                        if stop = '1' or power_bram_full = '1' or traces_bram_full = '1' or power_errors_count = POWER_MEASUREMENT_FAILURE then
                            -- When any bram is full move to Read state
                            state        <= S_READ;
                            capture_step <= S_ADC_START;
                        -- When no stop signal and the brams are not full... keep capturing
                        else
                            case capture_step is
                                -- Start an ADC measurement
                                when S_ADC_START =>
                                    capture_step <= S_ADC_WAIT;
                                -- Wait for the ADC to finish the measurement
                                when S_ADC_WAIT =>
                                    -- When the ADC finishes go back to Start state
                                    if adc_ready = '1' then
                                        capture_step <= S_ADC_START;
                                    end if;
                                -- Should never happen
                                when others =>
                                    capture_step <= S_ADC_START;
                            end case;
                        end if;
                    -- State where the system waits for the user to read the BRAM and signal it
                    when S_READ =>
                        if stop = '1' then
                            state <= S_CLEAR;
                        end if;
                    -- State where the counter is cleared as well as the ADC is configured to read from CH1 (when dual-channel mode)
                    when S_CLEAR =>
                        if ADC_DUAL = false then
                            state <= S_ADC_BUSY;
                        else
                            case clear_step is
                                -- Wait until ADC is ready and move to clear
                                when S_WAIT_READY =>
                                    if adc_ready = '1' then
                                        clear_step <= S_START_CLEAR;
                                    end if;
                                -- Start switching ADC channel (and clear counter)
                                when S_START_CLEAR =>
                                    clear_step <= S_GO_BUSY;
                                -- Go back to ADC busy state
                                when S_GO_BUSY =>
                                    state      <= S_ADC_BUSY;
                                    clear_step <= S_WAIT_READY;
                                -- Should never happen
                                when others =>
                                    clear_step <= S_WAIT_READY;
                            end case;
                        end if;
                    -- Should never happen
                    when others =>
                        state                <= S_ADC_BUSY;
                        capture_step         <= S_ADC_START;
                        clear_step           <= S_WAIT_READY;
                        user_config_vref_reg <= "00";
                        initial_conditions   <= '0';
                end case;
            end if;
        end process;
    end generate;

    -- General FSM without power monitoring
    power_fsm_disabled: if ADC_ENABLE = false generate

        fsm: process(clk, rst_n)
        begin
            -- Asynchronous reset
            if rst_n = '0' then
                -- Reset states
                state              <= S_IDLE;
                -- Reset user configuration register
                initial_conditions <= '0';
            -- Synchronous process
            elsif clk'event and clk = '1' then
                case state is
                    -- System is ready
                    when S_IDLE =>
                        -- Start capturing when the is a trigger or start
                        if internal_start = '1' then
                            -- Move to Capture state and indicate that initial conditions (probes) must be captured
                            state              <= S_CAPTURE;
                            initial_conditions <= '1';
                        end if;
                    -- Capture state (each of its steps are described by another FSM)
                    when S_CAPTURE =>
                        -- Clean initial conditions (just a pulse)
                        initial_conditions <= '0';
                        -- Keep capturing until stop or traces brams are full
                        if stop = '1' or traces_bram_full = '1' then
                            -- When any bram is full move to Read state
                            state <= S_READ;
                        end if;
                    -- State where the system waits for the user to read the BRAM and signal it
                    when S_READ =>
                        if stop = '1' then
                            state <= S_CLEAR;
                        end if;
                    -- State where the counter is cleared
                    when S_CLEAR =>
                        state <= S_IDLE;
                    -- Should never happen
                    when others =>
                        state                <= S_IDLE;
                        initial_conditions   <= '0';
                end case;
            end if;
        end process;
    end generate;

    -----------------
    -- FSM signals --
    -----------------

    -- Probes and AXI triggering enable
    trigger_enable  <= '1' when state = S_IDLE else '0';

    -- Counter and adc clear
    clear           <= '1' when ADC_ENABLE = false and state = S_CLEAR else
                       '1' when ADC_ENABLE = true and ADC_DUAL = true and state = S_CLEAR and clear_step = S_START_CLEAR else
                       '1' when ADC_ENABLE = true and ADC_DUAL = false and state = S_CLEAR else
                       '0';

    -- ADC enable
    acd_enable      <= '1' when ADC_ENABLE = true and state = S_CAPTURE and capture_step = S_ADC_START and power_bram_full = '0' and power_errors_count /= POWER_MEASUREMENT_FAILURE else '0';

    -- Traces event detection enable
    traces_enable   <= '1' when state = S_CAPTURE and traces_bram_full = '0' else '0';

    -- Status output
    busy            <= '0' when state = S_IDLE else '1';
    done            <= '1' when state = S_READ else '0';

    -------------
    -- Outputs --
    -------------

    -- Elapsed time in clock cycles
    count <= counter_count;

    ---------------------
    -- BRAM management --
    ---------------------
    -- - BRAMs addresses
    -- - BRAMs full flag
    -- - BRAMs utilization info

    power_bram_management_enabled: if ADC_ENABLE = true generate
        bram_address_management: process(clk, rst_n)
        begin

            -- Asynchronous reset
            if rst_n = '0' then
                power_bram_addr  <= (others => '0');
                power_bram_full  <= '0';
                power_bram_utilization <= (others => '0');
                traces_bram_addr <= (others => '0');
                traces_bram_full <= '0';
                traces_bram_utilization <= (others => '0');

            -- Synchronous process
            elsif clk'event and clk = '1' then
                if state = S_ADC_BUSY then
                    power_bram_addr  <= (others => '0');
                    power_bram_full  <= '0';
                    power_bram_utilization <= (others => '0');
                    traces_bram_addr <= (others => '0');
                    traces_bram_full <= '0';
                    traces_bram_utilization <= (others => '0');
                else
                    if power_bram_we_fix = '1' then
                        power_bram_utilization <= std_logic_vector(power_bram_addr);
                        if power_bram_addr = POWER_DEPTH - 1 then
                            power_bram_full <= '1';
                        else
                            power_bram_addr <= power_bram_addr + 1;
                        end if;
                    end if;
                    if traces_bram_we = '1' then
                        traces_bram_utilization <= std_logic_vector(traces_bram_addr);
                        if traces_bram_addr = TRACES_DEPTH - 1 then
                            traces_bram_full <= '1';
                        else
                            traces_bram_addr <= traces_bram_addr + 1;
                        end if;
                    end if;
                end if;
            end if;
        end process;

        -- Power BRAM write enable management
        power_bram_we  <= adc_ready when state = S_CAPTURE and capture_step = S_ADC_WAIT and power_bram_full = '0' and power_errors_count /= POWER_MEASUREMENT_FAILURE else
                          '0';

        -- Generate statement to create slightly different HW depending if the ADC works in dual or mono channel
        generate_power_error_fix_dual_mode: if ADC_DUAL = true generate
            power_bram_we_fix <= '1' when power_bram_we = '1' and ((power_bram_addr(0) = '0' and power_data(15 downto 12) = "0001") or (power_bram_addr(0) = '1' and power_data(15 downto 12) = "0011"))  else
                                 '0';
        end generate;
        generate_power_error_fix_single_mode: if ADC_DUAL = false generate
            power_bram_we_fix <= '1' when power_bram_we = '1' and power_data(15 downto 12) = "0000"  else
                                 '0';
        end generate;

        -- Error counter signal
        power_errors  <= power_errors_count;
        power_error_s <= '1' when power_bram_we = '1' and power_bram_we_fix = '0' else
                         '0';

    end generate;

    power_bram_management_disabled: if ADC_ENABLE = false generate
        bram_address_management: process(clk, rst_n)
        begin

            -- Asynchronous reset
            if rst_n = '0' then
                traces_bram_addr <= (others => '0');
                traces_bram_full <= '0';
                traces_bram_utilization <= (others => '0');

            -- Synchronous process
            elsif clk'event and clk = '1' then
                if state = S_IDLE then
                    traces_bram_addr <= (others => '0');
                    traces_bram_full <= '0';
                    traces_bram_utilization <= (others => '0');
                else
                    if traces_bram_we = '1' then
                        traces_bram_utilization <= std_logic_vector(traces_bram_addr);
                        if traces_bram_addr = TRACES_DEPTH - 1 then
                            traces_bram_full <= '1';
                        else
                            traces_bram_addr <= traces_bram_addr + 1;
                        end if;
                    end if;
                end if;
            end if;
        end process;
    end generate;

    -- Traces BRAM write enable management
    traces_bram_we <= '1'             when state = S_CAPTURE and initial_conditions = '1' else
                      event_detected  when state = S_CAPTURE and traces_bram_full = '0' else
                      '0';

    -- BRAM probes
    probes_bram_data  <= probes_delayed when state = S_CAPTURE and initial_conditions = '1' else
                         edges  when state = S_CAPTURE else
                         (others => '0');



end Behavioral;
