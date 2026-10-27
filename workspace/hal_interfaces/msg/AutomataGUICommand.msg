# AutomataGuiCommand.msg

# --- STAŁE (ENUM) ---
# Podstawowe komendy systemowe
uint8 STOP = 0
uint8 START = 1

# --- POLA WIADOMOŚCI ---
uint8 command          # Główna komenda (użyj stałych powyżej)
int32 high_state_id    # ID nadrzędnego stanu

# Parametry dynamiczne
float64[] float_params 
string[] string_params 