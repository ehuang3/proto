#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <locale>
#include <ncurses.h>

using namespace std;
using namespace Eigen;

WINDOW* create_window(int h, int w, int y, int x);
void destory_window(WINDOW* win);
void init_all_colors();
void wset_color(WINDOW* win, int c1, int c2);
void wunset_color(WINDOW* win, int c1, int c2);

// 6 colors
// 4 peg code
// max number guesses
// guess check algo
// enter master code
// need to see all guesses

// data structs
// 

std::map<int, std::string> CODE_PEG = 
{
    {0, "\033[1;31m 0 \033[0m"},
    {1, "\033[1;32m 0 \033[0m"},
    {2, "\033[1;33m 0 \033[0m"},
    {3, "\033[1;34m 0 \033[0m"},
    {4, "\033[1;35m 0 \033[0m"},
    {5, "\033[1;36m 0 \033[0m"},
};

static const int CODEX_MIN = 0;
static const int CODEX_MAX = 5;

std::map<int, std::string> KEY_PEG = 
{
    {0, "\033[1;31;47mo\033[0m"},
    {1, "\033[1;30;47mo\033[0m"},
};

enum MM_COLOR {
    MM_COLOR_BLACK   = 0,
    MM_COLOR_RED     = 1,
    MM_COLOR_GREEN   = 2,
    MM_COLOR_YELLOW  = 3,
    MM_COLOR_BLUE    = 4,
    MM_COLOR_MAGENTA = 5,
    MM_COLOR_CYAN    = 6,
    MM_COLOR_WHITE   = 7,
};

struct Code : public std::vector<int> {
protected:
    int code_len;
public:
    Code(int len=4)
        : code_len(len)
    {
        for(int i=0; i < code_len; i++) {
            push_back(0);
        }
    }

    int len() { return code_len; }
    bool full() { return code_len == size(); }

    bool operator==(const Code& b) {
        for(int i=0; i < size(); i++) {
            if(this->at(i) != b[i])
                return false;
        }
    }

    int rcrp(const Code& b) {
        int sum = 0;
        for(int i=0; i < size(); i++) {
            if(this->at(i) == b[i])
                sum++;
        }
        return sum;
    }

    int rcwp(const Code& b) {
        std::vector<int> colors;
        for(int i=0; i < size(); i++) {
            colors.push_back(0);
        }
        for(int i=0; i < size(); i++) {
            colors[this->at(i)]++;
        }
        int sum=0;
        for(int i=0; i < size(); i++) {
            if(colors[b[i]]-- > 0) {
                sum++;
            }
        }
        return sum - rcrp(b);
    }

    void mvprint(WINDOW* win, int y, int x) {
        wmove(win, y, x);
        print(win);
    }

    void print(WINDOW* win) {
        for(int i = 0; i != size(); i++) {
            wset_color(win, at(i)+1, COLOR_BLACK);
            wprintw(win, "0");
        }
    }

    void mvprint_secretly(WINDOW* win, int y, int x) {
        wmove(win, y, x);
        print_secretly(win);
    }
    
    void print_secretly(WINDOW* win) {
        for(int i = 0; i != size(); i++) {
            wset_color(win, COLOR_BLACK, COLOR_WHITE);
            wprintw(win, "*");
        }
    }

    void mvprint_diff(WINDOW* win, int y, int x, const Code& dc) {
        wset_color(win, COLOR_GREEN, COLOR_WHITE);
        for(int i=0; i < rcrp(dc); i++) {
            mvwprintw(win, y, x++, "o");
        }
        wset_color(win, COLOR_RED, COLOR_WHITE);
        for(int i=0; i < rcwp(dc); i++) {
            mvwprintw(win, y, x++, "o");
        }
    }
    
    void print_diff(WINDOW* win, const Code& dc) {
        wset_color(win, COLOR_GREEN, COLOR_WHITE);
        for(int i=0; i < rcrp(dc); i++) {
            wprintw(win, "o");
        }
        wset_color(win, COLOR_RED, COLOR_WHITE);
        for(int i=0; i < rcwp(dc); i++) {
            wprintw(win, "o");
        }
    }
};

class MasterMind {
public:
    std::vector<Code> decode;
    Code code;

public:
    MasterMind(int max) : max_attempts(max) {};

    // true if next turn should be played
    bool breakCode(Code guess) {
        decode.push_back(guess);
        if(guess == code || ++attempts >= max_attempts) {
            return false;
        }
    }

    bool cracked() {
        return *decode.end() == code;
    }

protected:
    int attempts;
    int max_attempts;
};

class CodeReader {
public:
    
};

WINDOW* create_window(int h, int w, int y, int x)
{
    WINDOW* new_win;
    new_win = newwin(h, w, y, x);
    box(new_win, 0, 0);
    wrefresh(new_win);
    return new_win;
}

void destory_window(WINDOW* win)
{
    wborder(win,' ',' ',' ',' ',' ',' ',' ',' ');
    wrefresh(win);
    delwin(win);
}

void init_all_colors()
{
    for(int i=MM_COLOR_BLACK; i <= MM_COLOR_WHITE; i++) {
        for(int j=MM_COLOR_BLACK; j <= MM_COLOR_WHITE; j++) {
            init_pair(1+i+j*8, i, j);
        }
    }
}

void wset_color(WINDOW* win, int c1, int c2)
{
    wattron(win, COLOR_PAIR(1+c1+8*c2));
}

void wunset_color(WINDOW* win, int c1, int c2)
{
    wattroff(win, COLOR_PAIR(1+c1+8*c2));
}

Code get_code(WINDOW* win, int height, const Code& prev) {
    Code code = prev;
    int ch;
    int x=1, y=height-1;
    wmove(win, y, x);
    code.print(win);
    wmove(win, y, x);
    wrefresh(win);
    while(ch = wgetch(win)) // Escape key
    {
        switch (ch) {
        case KEY_BACKSPACE:
            mvwprintw(win, y, x, " ");
            ch = KEY_LEFT;      // Move to left
            break;
        case KEY_UP:
            if(code[x-1] < CODEX_MAX)
                code[x-1]++;
            break;
        case KEY_DOWN:
            if(code[x-1] > CODEX_MIN)
                code[x-1]--;
            break;
        default:
            if('1'+CODEX_MIN <= ch && ch <= '1'+CODEX_MAX) {
                code[x-1] = ch-'1';
            }
            break;
        }
        switch (ch) {
        case KEY_RIGHT:
            if(x < code.len())
                x++;
            break;
        case KEY_LEFT:
            if(x > 1)
                x--;
            break;
        case 13:                // KEY_ENTER
            return code;
            break;
        }
        code.mvprint(win, y, 1);
        wmove(win, y, x);
        wrefresh(win);
    }
}

class CLASS {
	int c;
};

int main(int argc, char* argv[]) {

    int ch;
    WINDOW* win = initscr();
    start_color();
    init_all_colors();
    keypad(win, true);
    // raw();
    nonl();
    noecho();
    cbreak();
    clear();
    refresh();

    int height = 20;
    int width = 50;

    WINDOW* mm_win = create_window(height, width, (LINES-height)/2, (COLS-width)/2);
    keypad(mm_win, true);
    refresh();

    int N = 12;
    int i = 0;
    std::vector<Code> dc;
    
    Code mc;
    mc = get_code(mm_win, height, mc);
    int x=1, y=1;
    mc.mvprint_secretly(mm_win, y++, x);
    wrefresh(mm_win);
    Code c;
    while(i++ < N) {
        c = get_code(mm_win, height, c);
        dc.push_back(c);
        
        x=1, y=1;
        mc.mvprint_secretly(mm_win, y++, x);

        for(int i=0; i < dc.size(); i++) {
            dc[i].mvprint(mm_win, y, x);
            wprintw(mm_win, " ");
            mc.print_diff(mm_win, dc[i]);
            y++;
        }
        wrefresh(mm_win);
    }

    while(getch()) {};

    endwin();
    return 0;
}
