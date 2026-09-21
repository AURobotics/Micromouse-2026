#include "API.h"
#include <cstdlib>
#include <iostream>
#include <queue>

#define SIZE 16

void log(const std::string& text) {
    std::cerr << text << std::endl;
}

int dis[SIZE][SIZE];
bool walls[SIZE][SIZE][4]; // 0: North, 1: East, 2: South, 3: West
bool visited[SIZE][SIZE] = {false}; 

int curr_r = 15, curr_c = 0;
int curr_dir = 0;            

int dr[] = {-1, 0, 1, 0}; // 0: North, 1: East, 2: South, 3: West
int dc[] = {0, 1, 0, -1};

std::string getDir(int dir) {
    switch(dir) {
        case 0: return "North";
        case 1: return "East";
        case 2: return "South";
        case 3: return "West";
        default: return "Unknown";
    }
}


void updateWall(int r, int c, int dir) {
    walls[r][c][dir] = true;
    
    int nr = r + dr[dir];
    int nc = c + dc[dir];
  
    if (nr >= 0 && nr < SIZE && nc >= 0 && nc < SIZE) {
        int opposite_dir = (dir + 2) % 4;
        walls[nr][nc][opposite_dir] = true;
    }
}

void floodFill(int target_r, int target_c) {
    log("ray7a l cell (" + std::to_string(target_r) + ", " + std::to_string(target_c) + ")");
    
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            dis[i][j] = -1;
        }
    }

    std::queue<std::pair<int, int>> q;
    dis[target_r][target_c] = 0;
    q.push({target_r, target_c});

    while (!q.empty()) {
        auto [r, c] = q.front();
        q.pop();

        for (int i = 0; i < 4; i++) {
            int nr = r + dr[i];
            int nc = c + dc[i];

            if (nr >= 0 && nr < SIZE && nc >= 0 && nc < SIZE) {
                if (!walls[r][c][i] && dis[nr][nc] == -1) {
                    dis[nr][nc] = dis[r][c] + 1;
                    q.push({nr, nc});
                }
            }
        }
    }
}

void moveTo(int target_r, int target_c) {
    int target_dir = 0;
    if (target_r < curr_r) target_dir = 0;       
    else if (target_c > curr_c) target_dir = 1;  
    else if (target_r > curr_r) target_dir = 2;  
    else if (target_c < curr_c) target_dir = 3;  

    log("Moving from (" + std::to_string(curr_r) + ", " + std::to_string(curr_c) + 
        ") [Facing: " + getDir(curr_dir) + "] to (" + std::to_string(target_r) + ", " + std::to_string(target_c) + ")");

    int diff = (target_dir - curr_dir + 4) % 4;

    if (diff == 1) {
        API::turnRight();
        curr_dir = (curr_dir + 1) % 4;
    } else if (diff == 3) {
        API::turnLeft();
        curr_dir = (curr_dir + 3) % 4;
    } else if (diff == 2) {
        API::turnRight();
        API::turnRight();
        curr_dir = (curr_dir + 2) % 4;
    }

    API::moveForward();
    
    curr_r = target_r;
    curr_c = target_c;
    visited[curr_r][curr_c] = true;
    
    log("Arrived at: (" + std::to_string(curr_r) + ", " + std::to_string(curr_c) + ")");
}

bool isInCenter(int r, int c) {
    return ((r == 7 || r == 8) && (c == 7 || c == 8));
}

int main() {
    log("bsm llah el ra7man el ra7im"); 
    visited[curr_r][curr_c] = true;

    while (!isInCenter(curr_r, curr_c)) {
        log("Processing Cell: (" + std::to_string(curr_r) + ", " + std::to_string(curr_c) + ") ---");
        
      // wall detection
        bool f = API::wallFront();
        bool r = API::wallRight();
        bool l = API::wallLeft();

        if (f) updateWall(curr_r, curr_c, curr_dir);
        if (r) updateWall(curr_r, curr_c, (curr_dir + 1) % 4);
        if (l) updateWall(curr_r, curr_c, (curr_dir + 3) % 4);

        floodFill(7, 7);

        int best_r = curr_r, best_c = curr_c;
        int min_val = 1000;

        for (int i = 0; i < 4; i++) {
            int nr = curr_r + dr[i];
            int nc = curr_c + dc[i];
            if (nr >= 0 && nr < SIZE && nc >= 0 && nc < SIZE) {
                if (!walls[curr_r][curr_c][i] && dis[nr][nc] != -1) {
                    log("neighbor (" + std::to_string(nr) + ", " + std::to_string(nc) + 
                        ") [Dist: " + std::to_string(dis[nr][nc]) + ", Visited: " + std::to_string(visited[nr][nc]) + "]");

                    if (dis[nr][nc] < min_val) {
                        min_val = dis[nr][nc];
                        best_r = nr;
                        best_c = nc;
                    }
                }
            }
        }
        
        log("Best Next Cell: (" + std::to_string(best_r) + ", " + std::to_string(best_c) + ") [Dist: " + std::to_string(min_val) + "]");

        moveTo(best_r, best_c);
    }
    
    log("shmshon  wasl ll center el maze");
    return 0;
}