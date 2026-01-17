`default_nettype none

module gpu (
    input  logic        clk,
    input  logic        draw,
    input  logic [11:0] addr,
    input  logic [3:0]  lines,
    input  logic [5:0]  x,
    input  logic [4:0]  y,

    output logic        busy,
    output logic        collision,

    output logic        mem_read,
    output logic [11:0] mem_read_idx,
    input  logic [7:0]  mem_read_byte,
    input  logic        mem_read_ack,

    output logic        mem_write,
    output logic [11:0] mem_write_idx,
    output logic [7:0]  mem_write_byte
);

    // ----------------------------------------------------------------
    // Parameters
    // ----------------------------------------------------------------
    localparam int WIDTH = 8;

    typedef enum logic [2:0] {
        STATE_IDLE,
        STATE_LOAD_SPRITE,
        STATE_LOAD_MEM_LEFT,
        STATE_STORE_MEM_LEFT,
        STATE_LOAD_MEM_RIGHT,
        STATE_STORE_MEM_RIGHT
    } state_t;

    // ----------------------------------------------------------------
    // Registers
    // ----------------------------------------------------------------
    state_t      state;

    logic [3:0]  lines_left;
    logic [3:0]  shift;
    logic        use_right;

    logic [11:0] sprite_addr;
    logic [7:0]  screen_addr;

    logic [15:0] sprite_word;
    logic [7:0]  screen_byte;

    // ----------------------------------------------------------------
    // Derived addresses
    // ----------------------------------------------------------------
    logic [11:0] mem_idx_left;
    logic [11:0] mem_idx_right;

    assign mem_idx_left  = {4'h1, screen_addr};

    // wrap within an 8-byte row
    assign mem_idx_right = {
        4'h1,
        screen_addr[7:3],
        screen_addr[2:0] + 3'd1
    };

    // ----------------------------------------------------------------
    // Busy flag
    // ----------------------------------------------------------------
    assign busy = (state != STATE_IDLE);

    // ----------------------------------------------------------------
    // Memory control (combinational)
    // ----------------------------------------------------------------
    always_comb begin
        mem_read       = 1'b0;
        mem_read_idx   = '0;
        mem_write      = 1'b0;
        mem_write_idx  = '0;
        mem_write_byte = '0;

        case (state)
            STATE_LOAD_SPRITE: begin
                if (!mem_read_ack) begin
                    mem_read     = 1'b1;
                    mem_read_idx = sprite_addr;
                end
            end

            STATE_LOAD_MEM_LEFT: begin
                if (!mem_read_ack) begin
                    mem_read     = 1'b1;
                    mem_read_idx = mem_idx_left;
                end
            end

            STATE_LOAD_MEM_RIGHT: begin
                if (!mem_read_ack) begin
                    mem_read     = 1'b1;
                    mem_read_idx = mem_idx_right;
                end
            end

            STATE_STORE_MEM_LEFT: begin
                mem_write      = 1'b1;
                mem_write_idx  = mem_idx_left;
                mem_write_byte = screen_byte;
            end

            STATE_STORE_MEM_RIGHT: begin
                if (use_right) begin
                    mem_write      = 1'b1;
                    mem_write_idx  = mem_idx_right;
                    mem_write_byte = screen_byte;
                end
            end

            default: ;
        endcase
    end

    // ----------------------------------------------------------------
    // State machine
    // ----------------------------------------------------------------
    always_ff @(posedge clk) begin
        case (state)
            STATE_IDLE: begin
                if (draw) begin
                    lines_left  <= lines - 1'b1;
                    sprite_addr <= addr;
                    screen_addr <= y * WIDTH + (x >> 3);
                    shift       <= x[2:0];
                    use_right   <= (x[2:0] != 3'd0);
                    collision   <= 1'b0;
                    state       <= STATE_LOAD_SPRITE;
                end
            end

            STATE_LOAD_SPRITE: begin
                if (mem_read_ack) begin
                    sprite_word <= {mem_read_byte, 8'b0} >> shift;
                    state       <= STATE_LOAD_MEM_LEFT;
                end
            end

            STATE_LOAD_MEM_LEFT: begin
                if (mem_read_ack) begin
                    screen_byte <= mem_read_byte ^ sprite_word[15:8];
                    collision   <= collision |
                                   |(mem_read_byte & sprite_word[15:8]);
                    state       <= STATE_STORE_MEM_LEFT;
                end
            end

            STATE_STORE_MEM_LEFT: begin
                state <= STATE_LOAD_MEM_RIGHT;
            end

            STATE_LOAD_MEM_RIGHT: begin
                if (mem_read_ack) begin
                    if (use_right) begin
                        screen_byte <= mem_read_byte ^ sprite_word[7:0];
                        collision   <= collision |
                                       |(mem_read_byte & sprite_word[7:0]);
                    end
                    state <= STATE_STORE_MEM_RIGHT;
                end
            end

            STATE_STORE_MEM_RIGHT: begin
                if (lines_left == 0) begin
                    state <= STATE_IDLE;
                end
                else begin
                    sprite_addr <= sprite_addr + 12'd1;
                    screen_addr <= screen_addr + WIDTH;
                    lines_left  <= lines_left - 1'b1;
                    state       <= STATE_LOAD_SPRITE;
                end
            end

            default: state <= STATE_IDLE;
        endcase
    end

endmodule
