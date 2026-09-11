import os

file_path = "main.cpp"
with open(file_path, "r", encoding="utf-8") as f:
    content = f.read()

# Revert the SLEEP UART send logic
sleep_uart_logic = """
                ESP_LOGI(TAG, "Deep sleep button pressed. Going to sleep.");
                
                // Notify TDOA board to sleep as well (cross-board sync)
                const char* sleep_cmd = "SLEEP\\n";
                uart_write_bytes(ANGLE_UART_PORT, sleep_cmd, strlen(sleep_cmd));
                uart_wait_tx_done(ANGLE_UART_PORT, pdMS_TO_TICKS(100));
"""

original_logic = '                ESP_LOGI(TAG, "Deep sleep button pressed. Going to sleep.");'

content = content.replace(sleep_uart_logic, original_logic)

with open(file_path, "w", encoding="utf-8") as f:
    f.write(content)
print("Reverted SLEEP sync patch.")
