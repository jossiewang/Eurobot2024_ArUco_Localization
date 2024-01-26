#include <iostream>
#include <cstdio>

int main() {
    // 等待使用者按下Enter
    std::cout << "按下Enter將\"abs\"存入剪貼簿...";
    while (std::cin.get() != '\n');

    // 使用 popen 命令執行 xclip 命令
    FILE* clipboard = popen("xclip -selection clipboard", "w");
    if (clipboard != NULL) {
        // 寫入資料到剪貼簿
        fprintf(clipboard, "abs");
        
        // 關閉文件指針
        pclose(clipboard);
    } else {
        std::cerr << "無法打開 xclip 命令。請確保已安裝 xclip 工具。\n";
        return 1;
    }

    std::cout << "已將\"abs\"存入剪貼簿。\n";
    return 0;
}
