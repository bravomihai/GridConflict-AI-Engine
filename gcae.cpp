#include "gcae.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

const int INF = std::numeric_limits<int>::max() / 4;

/* indexToRowChar
 - Convert numeric row index (0..51) into encoded character ('A'..'Z', 'a'..'z').
 - Used when serializing board coordinates into the compact state string. */
static inline char indexToRowChar(int idx)
{
    if (idx < 26)
        return static_cast<char>('A' + idx);
    return static_cast<char>('a' + (idx - 26));
}

/* rowCharToIndex
 - Convert encoded row character back to numeric index.
 - Returns -1 for invalid characters. */
static inline int rowCharToIndex(char c)
{
    if (c >= 'A' && c <= 'Z')
        return c - 'A';
    if (c >= 'a' && c <= 'z')
        return (c - 'a') + 26;
    return -1;
}

/* encode
 - Serialize a 2D board into the engine's compact state string.
 - Emits tokens only for letters/digits, prefixed with 'o' for numeric objects. */
void encode(int H, int W, const std::vector<std::vector<char>> &board, std::string &out)
{
    out.clear();
    for (int r = 0; r < H; ++r)
    {
        for (int c = 0; c < W; ++c)
        {
            char ch = board[r][c];
            if (!(std::isdigit(static_cast<unsigned char>(ch)) || std::isupper(static_cast<unsigned char>(ch)) ||
                  std::islower(static_cast<unsigned char>(ch))))
                continue;
            if (std::isdigit(static_cast<unsigned char>(ch)))
                out.push_back('o');
            out.push_back(ch);
            out.push_back(' ');
            out.push_back(indexToRowChar(r));
            int col = c + 1;
            if (col >= 10)
                out.push_back(char('0' + (col / 10) % 10));
            out.push_back(char('0' + (col % 10)));
            out.push_back(' ');
        }
    }
}

/* decode
 - Parse the compact state string and populate the board matrix.
 - Handles 1- and 2-digit columns and optional 'o' prefixes; defensive to malformed tokens. */
void decode(int H, int W, std::vector<std::vector<char>> &board, const std::string &s)
{
    for (int i = 0; i < H; ++i)
        board[i].assign(W, '.');

    size_t i = 0, n = s.size();
    while (i < n)
    {
        if (s[i] == ' ')
        {
            ++i;
            continue;
        }
        bool is_obj = false;
        if (s[i] == 'o')
        {
            is_obj = true;
            ++i;
            if (i >= n)
                break;
        }
        char ent = s[i++]; // entity char
        if (i < n && s[i] == ' ')
            ++i;
        if (i >= n)
            break;
        char rowChar = s[i++];
        int row = rowCharToIndex(rowChar);
        if (row < 0)
            continue;
        if (i >= n)
            break;
        int col = s[i++] - '0';
        if (i < n && std::isdigit(static_cast<unsigned char>(s[i])))
        {
            col = col * 10 + (s[i++] - '0');
        }
        if (row >= 0 && row < H && col >= 1 && col <= W)
        {
            board[row][col - 1] = ent;
        }
        if (i < n && s[i] == ' ')
            ++i;
    }
}

/* getrow
 - Read a single row character from encoded token at pos and advance pos.
 - Lightweight helper used by parsing routines. */
static inline char getrow(const std::string &str, int &pos)
{
    char c = str[pos];
    ++pos;
    return c;
}

/* getcol
 - Read a 1- or 2-digit column from encoded string at pos and advance pos.
 - Supports columns up to 99. */
static inline int getcol(const std::string &str, int &pos)
{
    int col = 0;
    if (pos + 1 < (int)str.size() && std::isdigit(static_cast<unsigned char>(str[pos + 1])))
    {
        col = (str[pos++] - '0') * 10;
    }
    col += (str[pos++] - '0');
    return col;
}

/* add_c
 - Insert `len` null characters at `start` in the string (bounds-checked).
 - Thin wrapper kept for compatibility with original logic. */
static inline int add_c(std::string &s, int start, int len)
{
    if (start < 0 || len <= 0 || start > (int)s.size())
        return -1;
    s.insert((size_t)start, std::string(len, '\0'));
    return 1;
}

/* delete_c
 - Remove `len` characters from `start` in the string (bounds-safe).
 - Used to remove object/monster tokens from the encoded state. */
static inline int delete_c(std::string &s, int start, int len)
{
    if (start < 0 || len <= 0 || start > (int)s.size())
        return -1;
    if (start + len > (int)s.size())
        len = (int)s.size() - start;
    s.erase((size_t)start, (size_t)len);
    return 1;
}

/* distance_rc
 - Return Manhattan distance between two encoded positions (row char + column).
 - Converts row characters to numeric indices first. */
static inline int distance_rc(char row, int col, char row2, int col2)
{
    int r1 = rowCharToIndex(row), r2 = rowCharToIndex(row2);
    int vr = (r1 > r2) ? r1 - r2 : r2 - r1;
    int vc = (col > col2) ? col - col2 : col2 - col;
    return vr + vc;
}

/* closest_valid_point
 - Compute the nearest reachable valid position toward a target within at most `s` movement points.
 - A position is valid if it lies within board bounds and is not already occupied in the encoded state string.
 - Progressively relaxes the path allocation between primary and secondary movement budgets until a valid tile is found.
 */
static point closest_valid_point(char row, int col, char cp_row, int cp_col, int s, const std::string &str, int H,
                                 int W)
{
    /* compute_closest
     - Internal heuristic that advances from the current position toward the target.
     - Uses two movement budgets: primary (direct approach) and secondary (adjustments).
     - Returns the resulting intermediate point without validating occupancy. */
    auto compute_closest = [&](int s1, int s2) -> point {
        int target_row = rowCharToIndex(row);
        int start_row = rowCharToIndex(cp_row);

        int cur_row = start_row;
        int cur_col = cp_col;

        // primary directional movement
        while (cur_row < target_row && s1)
        {
            ++cur_row;
            --s1;
        }
        while (cur_col < col && s1)
        {
            ++cur_col;
            --s1;
        }
        while (cur_row > target_row && s1)
        {
            --cur_row;
            --s1;
        }
        while (cur_col > col && s1)
        {
            --cur_col;
            --s1;
        }

        // secondary adjustments
        while (cur_col < col && s2)
        {
            ++cur_col;
            --s2;
        }
        while (cur_col > col && s2)
        {
            --cur_col;
            --s2;
        }
        while (cur_row < target_row && s2)
        {
            ++cur_row;
            --s2;
        }
        while (cur_row > target_row && s2)
        {
            --cur_row;
            --s2;
        }

        point p;
        p.row = indexToRowChar(cur_row);
        p.col = cur_col;
        return p;
    };

    auto is_valid = [&](const point &p) -> bool {
        int ridx = rowCharToIndex(p.row);
        if (!(ridx >= 0 && ridx < H && p.col > 0 && p.col <= W))
            return false;

        std::string sub;
        sub.push_back(p.row);
        if (p.col > 9)
            sub.push_back(char('0' + (p.col / 10) % 10));
        sub.push_back(char('0' + (p.col % 10)));

        return (str.find(sub) == std::string::npos);
    };

    // Initial attempt using full primary budget
    point closest = compute_closest(s, 0);

    // If invalid, progressively rebalance primary/secondary budgets
    while (!is_valid(closest) && s > 0)
    {
        int s2 = 0;
        while (!is_valid(closest) && s2 <= s)
        {
            closest = compute_closest(s - s2, s2);
            ++s2;
        }
        --s;
    }

    return closest;
}

static inline short maxim(short a, short b)
{
    return (a > b) ? a : b;
}

/* isobject_index
 - Determine whether the byte at index i is an object prefix ('o') in the encoded string.
 - Avoid false positives near other tokens. */
static bool isobject_index(int i, const std::string &s)
{
    if (i < 0 || i >= (int)s.size())
        return false;
    if (s[i] != 'o')
        return false;
    if (i == 0)
        return true;
    if (i - 2 >= 0)
    {
        char prev = s[i - 2];
        if (prev == 'm' || prev == 'A' || prev == 'B')
            return false;
    }
    return true;
}

/* end_round_state
 - Produce a copy of src into dest and set move to 'pass' type.
 - Used to represent end-of-turn/no-op successor. */
static void end_round_state(game_state &dest, const game_state &src, Move &m, int cp, int op)
{
    m.type = 'p';
    m.torow = '.';
    m.tocol = 0;
    dest = src;
}

/* move_player_apply
 - Update the player's encoded position inside the provided dest.game_state string.
 - Operates on current dest.s so that any prior deletions/insertions are respected. */
static bool move_player_apply(game_state &dest, Move &m, char row, int col, char current_player)
{
    m.type = 'm';
    m.torow = row;
    m.tocol = (short)col;

    std::string key;
    key.push_back(current_player);
    key.push_back(' ');
    size_t pos = dest.s.find(key);
    if (pos == std::string::npos)
        return false;

    size_t start = pos + 2;
    size_t end = start;
    while (end < dest.s.size() && dest.s[end] != ' ')
        ++end;

    std::string repl;
    repl.push_back(row);
    repl += std::to_string(col);

    dest.s.replace(start, end - start, repl);

    return true;
}

/* next_states
 - Generate successor game states for the player `current_player`.
 - Produces moves, attacks, pickups, and pass states.
 - Ensures string updates are applied before storing each successor. */
int next_states(int H, int W, const game_state &gs, char current_player, const std::vector<item> &items,
                std::vector<game_state> &ngs, std::vector<Move> &moves)
{
    ngs.clear();
    moves.clear();
    ngs.resize(30);
    moves.resize(30);

    int state = 0;
    int cp = (current_player == 'A') ? 0 : 1;
    int op = 1 - cp;
    char opp_char = (current_player == 'A') ? 'B' : 'A';
    if (gs.players[cp].s == 0)
    {
        end_round_state(ngs[state], gs, moves[state], cp, op);
        return 1;
    }

    size_t cp_pos = gs.s.find(std::string(1, current_player) + " ");
    if (cp_pos == std::string::npos)
    {
        end_round_state(ngs[state], gs, moves[state], cp, op);
        return 1;
    }
    int tmp = (int)cp_pos + 2;
    char cp_row = getrow(gs.s, tmp);
    int cp_col = getcol(gs.s, tmp);

    int o = 0, m = 0;
    for (int i = 0; i < (int)gs.s.size(); ++i)
    {
        if (gs.s[i] == 'm' && i + 1 < (int)gs.s.size() && gs.s[i + 1] == ' ')
            ++m;
        if (isobject_index(i, gs.s))
            ++o;
    }

    size_t opp_pos = gs.s.find(std::string(1, opp_char) + " ");
    if (opp_pos == std::string::npos)
    {
        end_round_state(ngs[state], gs, moves[state], cp, op);
        return 1;
    }
    tmp = (int)opp_pos + 2;
    char opp_row = getrow(gs.s, tmp);
    int opp_col = getcol(gs.s, tmp);

    int dtop = distance_rc(opp_row, opp_col, cp_row, cp_col);

    if (dtop == 1)
    {
        if (gs.players[cp].s >= 10)
        {
            Move mm;
            mm.type = 'a';
            mm.torow = opp_row;
            mm.tocol = (short)opp_col;
            ngs[state] = gs;
            short damage = std::max<short>(0, gs.players[cp].A - gs.players[op].D);
            ngs[state].players[op].H = (short)(gs.players[op].H - damage);
            ngs[state].players[cp].s = (short)(gs.players[cp].s - 10);
            moves[state] = mm;
            ++state;
        }
        else
        {
            int di[] = {-1, 1, -1, 1};
            int dj[] = {-1, 1, 1, -1};
            for (int dir = 0; dir < 4; ++dir)
            {
                char nr = (char)(opp_row + di[dir]);
                int nc = opp_col + dj[dir];
                if (nr == 'Z' + 1)
                    nr = 'a';
                if (nr == 'a' - 1)
                    nr = 'Z';
                int ri = rowCharToIndex(nr);
                bool in_bounds = (ri >= 0 && ri < H && nc > 0 && nc <= W);
                bool not_occupied = true;
                if (in_bounds)
                {
                    std::string sub;
                    sub.push_back(nr);
                    if (nc > 9)
                        sub.push_back(char('0' + (nc / 10) % 10));
                    sub.push_back(char('0' + (nc % 10)));
                    not_occupied = (gs.s.find(sub) == std::string::npos);
                }
                if (in_bounds && not_occupied)
                {
                    int dist = distance_rc(nr, nc, cp_row, cp_col);
                    if (dist <= gs.players[cp].s)
                    {
                        Move mm;
                        game_state dest = gs;
                        if (move_player_apply(dest, mm, nr, nc, current_player))
                        {
                            dest.players[cp].s = (short)(gs.players[cp].s - dist);
                            ngs[state] = std::move(dest);
                            mm.type = 'm';
                            moves[state] = mm;
                            ++state;
                            break;
                        }
                    }
                }
            }
        }
    }
    else
    {
        int di[] = {0, 0, 1, -1};
        int dj[] = {1, -1, 0, 0};
        for (int dir = 0; dir < 4; ++dir)
        {
            char nr = (char)(opp_row + di[dir]);
            int nc = opp_col + dj[dir];
            if (nr == 'Z' + 1)
                nr = 'a';
            if (nr == 'a' - 1)
                nr = 'Z';
            int ri = rowCharToIndex(nr);
            bool in_bounds = (ri >= 0 && ri < H && nc > 0 && nc <= W);
            bool not_occupied = true;
            if (in_bounds)
            {
                std::string sub;
                sub.push_back(nr);
                if (nc > 9)
                    sub.push_back(char('0' + (nc / 10) % 10));
                sub.push_back(char('0' + (nc % 10)));
                not_occupied = (gs.s.find(sub) == std::string::npos);
            }
            if (in_bounds && not_occupied)
            {
                int dist = distance_rc(nr, nc, cp_row, cp_col);
                if (dist + 10 <= gs.players[cp].s)
                {
                    Move mm;
                    game_state dest = gs;
                    if (move_player_apply(dest, mm, nr, nc, current_player))
                    {
                        dest.players[cp].s = (short)(gs.players[cp].s - dist);
                        ngs[state] = std::move(dest);
                        mm.type = 'm';
                        moves[state] = mm;
                        ++state;
                        break;
                    }
                }
                else if (dtop - 1 > gs.players[cp].s)
                {
                    point closest = closest_valid_point(nr, nc, cp_row, cp_col, gs.players[cp].s, gs.s, H, W);
                    int d = distance_rc(closest.row, closest.col, cp_row, cp_col);
                    std::string sub;
                    sub.push_back(closest.row);
                    if (closest.col > 9)
                        sub.push_back(char('0' + (closest.col / 10) % 10));
                    sub.push_back(char('0' + (closest.col % 10)));
                    bool val = (gs.s.find(sub) == std::string::npos);
                    if (val && d <= gs.players[cp].s)
                    {
                        Move mm;
                        game_state dest = gs;
                        if (move_player_apply(dest, mm, closest.row, closest.col, current_player))
                        {
                            dest.players[cp].s = 0;
                            ngs[state] = std::move(dest);
                            mm.type = 'm';
                            moves[state] = mm;
                            ++state;
                        }
                    }
                    else
                    {
                        end_round_state(ngs[state], gs, moves[state], cp, op);
                        ++state;
                    }
                }
                else
                {
                    end_round_state(ngs[state], gs, moves[state], cp, op);
                    ++state;
                    break;
                }
            }
        }
    }

    int i = 0;
    while (state <= o + 1)
    {
        while (i < (int)gs.s.size() && !isobject_index(i, gs.s))
            ++i;
        if (i >= (int)gs.s.size())
            break;
        int nr_o = (int)(gs.s[i + 1] - '0');
        i += 3;
        int start = i - 3, end = i + 1;
        while (end - 1 < (int)gs.s.size() && gs.s[end - 1] != ' ' && gs.s[end - 1] != '\0')
            ++end;
        char orow = getrow(gs.s, i);
        int ocol = getcol(gs.s, i);
        int dist = distance_rc(orow, ocol, cp_row, cp_col);
        if (dist <= gs.players[cp].s)
        {
            game_state dest = gs;
            if (delete_c(dest.s, start, end - start) == -1)
                return -1;
            Move mm;
            mm.type = 'm';
            mm.torow = orow;
            mm.tocol = (short)ocol;
            if (!move_player_apply(dest, mm, orow, ocol, current_player))
                return -1;
            dest.players[cp].s = (short)(gs.players[cp].s - dist);
            dest.players[cp].H = (short)(gs.players[cp].H + items[nr_o].dH);
            dest.players[cp].A = (short)(gs.players[cp].A + items[nr_o].dA);
            dest.players[cp].D = (short)(gs.players[cp].D + items[nr_o].dD);
            dest.players[cp].S = (short)(gs.players[cp].S + items[nr_o].dS);
            ngs[state] = std::move(dest);
            moves[state] = mm;
        }
        else
        {
            point closest = closest_valid_point(orow, ocol, cp_row, cp_col, gs.players[cp].s, gs.s, H, W);
            std::string sub;
            sub.push_back(closest.row);
            if (closest.col > 9)
                sub.push_back(char('0' + (closest.col / 10) % 10));
            sub.push_back(char('0' + (closest.col % 10)));
            if (gs.s.find(sub) == std::string::npos)
            {
                Move mm;
                game_state dest = gs;
                if (move_player_apply(dest, mm, closest.row, closest.col, current_player))
                {
                    dest.players[cp].s = 0;
                    ngs[state] = std::move(dest);
                    moves[state] = mm;
                }
            }
            else
            {
                end_round_state(ngs[state], gs, moves[state], cp, op);
            }
        }
        ++state;
    }

    i = 0;
    while (state <= o + m + 1)
    {
        while (i < (int)gs.s.size() && !(gs.s[i] == 'm' && i + 1 < (int)gs.s.size() && gs.s[i + 1] == ' '))
            ++i;
        if (i >= (int)gs.s.size())
            break;
        i += 2;
        int start = i - 2, end = i + 1;
        while (end - 1 < (int)gs.s.size() && gs.s[end - 1] != ' ' && gs.s[end - 1] != '\0')
            ++end;
        char mrow = getrow(gs.s, i);
        int mcol = getcol(gs.s, i);
        int dtom = distance_rc(mrow, mcol, cp_row, cp_col);
        if (dtom == 1 && gs.players[cp].s >= 10)
        {
            game_state dest = gs;
            if (delete_c(dest.s, start, end - start) == -1)
                return -1;
            Move mm;
            mm.type = 'a';
            mm.torow = mrow;
            mm.tocol = (short)mcol;
            dest.players[cp].s = (short)(gs.players[cp].s - 10);
            dest.players[cp].H = (short)(gs.players[cp].H + 10);
            ngs[state] = std::move(dest);
            moves[state] = mm;
            ++state;
        }
        else
        {
            int di[] = {0, 0, 1, -1};
            int dj[] = {1, -1, 0, 0};
            bool progressed = false;
            for (int dir = 0; dir < 4; ++dir)
            {
                char nr = (char)(mrow + di[dir]);
                int nc = mcol + dj[dir];
                if (nr == 'Z' + 1)
                    nr = 'a';
                if (nr == 'a' - 1)
                    nr = 'Z';
                int ri = rowCharToIndex(nr);
                bool in_bounds = (ri >= 0 && ri < H && nc > 0 && nc <= W);
                std::string sub;
                sub.push_back(nr);
                if (nc > 9)
                    sub.push_back(char('0' + (nc / 10) % 10));
                sub.push_back(char('0' + (nc % 10)));
                if (in_bounds && gs.s.find(sub) == std::string::npos)
                {
                    int dist = distance_rc(nr, nc, cp_row, cp_col);
                    if (dist <= gs.players[cp].s)
                    {
                        Move mm;
                        game_state dest = gs;
                        if (move_player_apply(dest, mm, nr, nc, current_player))
                        {
                            dest.players[cp].s = (short)(gs.players[cp].s - dist);
                            ngs[state] = std::move(dest);
                            moves[state] = mm;
                            ++state;
                            progressed = true;
                            break;
                        }
                    }
                    else if (dtom > gs.players[cp].s)
                    {
                        point closest = closest_valid_point(nr, nc, cp_row, cp_col, gs.players[cp].s, gs.s, H, W);
                        std::string subs;
                        subs.push_back(closest.row);
                        if (closest.col > 9)
                            subs.push_back(char('0' + (closest.col / 10) % 10));
                        subs.push_back(char('0' + (closest.col % 10)));
                        if (gs.s.find(subs) == std::string::npos)
                        {
                            Move mm;
                            game_state dest = gs;
                            if (move_player_apply(dest, mm, closest.row, closest.col, current_player))
                            {
                                dest.players[cp].s = 0;
                                ngs[state] = std::move(dest);
                                moves[state] = mm;
                            }
                        }
                        else
                        {
                            end_round_state(ngs[state], gs, moves[state], cp, op);
                        }
                        ++state;
                        progressed = true;
                        break;
                    }
                }
            }
            (void)progressed;
        }
    }

    end_round_state(ngs[state], gs, moves[state], cp, op);
    ++state;
    ngs.resize(state);
    moves.resize(state);
    return state;
}

/* game_over_check
 - Terminal test: either player's health <= 0 indicates game over. */
static inline bool game_over_check(const game_state &gs)
{
    return gs.players[0].H <= 0 || gs.players[1].H <= 0;
}

/* static_eval
 - Heuristic evaluation from perspective of root_player.
 - Returns +/-INF for decisive terminal states; otherwise a linear stat difference. */
static int static_eval(const game_state &gs, char root_player)
{
    int p = (root_player == 'A') ? 0 : 1;
    int o = 1 - p;

    if (game_over_check(gs) && gs.players[p].H > 0)
        return INF;
    if (game_over_check(gs) && gs.players[p].H <= 0)
        return -INF;

    int dmg_p = std::max(1, gs.players[p].A - gs.players[o].D);
    int dmg_o = std::max(1, gs.players[o].A - gs.players[p].D);

    int turns_to_kill_o = (gs.players[o].H + dmg_p - 1) / dmg_p;
    int turns_to_kill_p = (gs.players[p].H + dmg_o - 1) / dmg_o;

    int tempo_score = (turns_to_kill_p - turns_to_kill_o) * 100;

    int stat_score =
        gs.players[p].H + gs.players[p].A + gs.players[p].D + gs.players[p].S -
        (gs.players[o].H + gs.players[o].A + gs.players[o].D + gs.players[o].S);

    return tempo_score + stat_score;
}

/* close_game_eval
 - Evaluates a position when the game is forced to end due to too many consecutive passes.
 - The winner is decided by comparing the overall stats of both players.
 - Returns +INF if the root_player has the advantage, otherwise -INF. */
static int close_game_eval(const game_state &gs, char root_player){
    int p = (root_player == 'A') ? 0 : 1;
    int o = 1 - p;

    int stat_score =
        gs.players[p].H + gs.players[p].A + gs.players[p].D + gs.players[p].S -
        (gs.players[o].H + gs.players[o].A + gs.players[o].D + gs.players[o].S);

    if(stat_score > 0){
        return INF;
    }
    return -INF;

}

/* minimax_search
 - Depth-limited minimax with alpha-beta pruning.
 - current_player is the player to move at this node; root_player is the evaluation perspective.
 - Respects move types and stamina to decide depth progression. */
static int minimax_search(const game_state &gs, int depth, char current_player, char root_player, int H, int W,
                          const std::vector<item> &items, int &alpha, int &beta, int consecutivePasses, int &maxpasses)
{
    if(consecutivePasses >= maxpasses){
        return close_game_eval(gs, root_player);
    }

    if (depth == 0 || game_over_check(gs))
        return static_eval(gs, root_player);

    std::vector<game_state> ngs;
    std::vector<Move> moves;

    int nStates = next_states(H, W, gs, current_player, items, ngs, moves);

    if (nStates == 0)
        return static_eval(gs, root_player);

    bool maximizing = (current_player == root_player);

    if (maximizing)
    {
        int max_eval = -INF;

        for (int i = 0; i < nStates; ++i)
        {
            char next_player = current_player;

            if (moves[i].type == 'p'){
                next_player = (current_player == 'A') ? 'B' : 'A';
                consecutivePasses++;
            }
            else{
                consecutivePasses = 0;
            }

            int next_depth = (moves[i].type == 'p') ? depth - 1 : depth;

            int eval = minimax_search(ngs[i], next_depth, next_player, root_player, H, W, items, alpha, beta, consecutivePasses, maxpasses);

            max_eval = std::max(max_eval, eval);
            alpha = std::max(alpha, eval);

            if (beta <= alpha)
                break;
        }

        return max_eval;
    }
    else
    {
        int min_eval = INF;

        for (int i = 0; i < nStates; ++i)
        {
            char next_player = current_player;

            if (moves[i].type == 'p'){
                next_player = (current_player == 'A') ? 'B' : 'A';
                consecutivePasses++;
            }
            else{
                consecutivePasses = 0;
            }

            int next_depth = (moves[i].type == 'p') ? depth - 1 : depth;

            int eval = minimax_search(ngs[i], next_depth, next_player, root_player, H, W, items, alpha, beta, consecutivePasses, maxpasses);

            min_eval = std::min(min_eval, eval);
            beta = std::min(beta, eval);

            if (beta <= alpha)
                break;
        }

        return min_eval;
    }
}

EngineResult best_move(const char* file_name)
{
    std::ifstream fin(file_name);
    if (!fin.is_open())
    {
        Move m;
        m.type = 'p';
        m.torow = '.';
        m.tocol = 0;
        return {m, -INF, 0};
    }

    return best_move_from_stream(fin);
}

double score_to_chance(int score) // sigmoid function
{
    const double K = 200.0;   // scaling factor

    if(score >= INF) return 1.0;
    if(score <= -INF) return 0.0;

    return 1.0 / (1.0 + std::exp(-score / K));
}

/* best_move
 - Parse input file, run next_states + minimax, return best Move.
 - current_player is the player that must act now. */
EngineResult best_move_from_stream(std::istream& fin)
{
    EngineResult nullRes = {{'p', '.', 0}, -INF, 0};

    int H, W, depth;
    char current_player;
    fin >> H >> W >> current_player >> depth;
    if (!fin)
        return nullRes;

    game_state gs;
    fin >> gs.players[0].H >> gs.players[0].A >> gs.players[0].D >> gs.players[0].s >> gs.players[0].S;
    fin >> gs.players[1].H >> gs.players[1].A >> gs.players[1].D >> gs.players[1].s >> gs.players[1].S;

    int n;
    fin >> n;
    std::vector<item> items(n);
    for (int i = 0; i < n; ++i)
    {
        fin >> items[i].dH >> items[i].dA >> items[i].dD >> items[i].dS;
    }

    std::string rest;
    std::getline(fin, rest); // eat endline
    std::getline(fin, gs.s);

    if (!gs.s.empty() && gs.s.back() == '\r')
        gs.s.pop_back();

    std::vector<game_state> ngs;
    std::vector<Move> moves;

    int nStates = next_states(H, W, gs, current_player, items, ngs, moves);
    if (nStates <= 0)
        return nullRes;

    int best_index = 0;
    int best_score = -INF;

    for (int i = 0; i < nStates; ++i)
    {
        int alpha = -INF, beta = INF, consecutivePasses = 0, maxpasses = std::min(depth, 10);
        int score = minimax_search(
            ngs[i],
            depth,
            current_player,
            current_player,
            H, W,
            items,
            alpha, beta,
            consecutivePasses,
            maxpasses
        );

        if (score > best_score)
        {
            best_score = score;
            best_index = i;
        }
    }

    return {moves[best_index], best_score, score_to_chance(best_score)};
}