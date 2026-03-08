#ifndef GCAE_HPP
#define GCAE_HPP

#include <string>
#include <vector>

struct player
{
    short H, A, D, s, S;
};

struct item
{
    short dH, dA, dD, dS;
};

struct point
{
    char row;
    int col;
};

struct Move
{
    char type;   // 'm', 'a', 'p'
    char torow;  // row letter: 'A'..'Z' or 'a'..'z' (or '.' for pass)
    short tocol; // 1-based column (or 0 for pass)
};

struct EngineResult {
    Move move;
    int score;
    double winChance;
};

struct game_state
{
    player players[2];
    std::string s; // encoded map string
};

// Public utilities (optional but useful for tests)
void encode(int H, int W, const std::vector<std::vector<char>> &board, std::string &out);
void decode(int H, int W, std::vector<std::vector<char>> &board, const std::string &in);

// Core API
// Reads file_name, returns the best move, move score and win chance with the move for the current player.
EngineResult best_move_from_stream(std::istream& in);

int next_states(int H, int W, const game_state &gs, char next_player, const std::vector<item> &items,
                std::vector<game_state> &ngs, std::vector<Move> &moves);

#endif // GCAE_HPP