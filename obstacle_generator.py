import random
import math

# --- CONFIGURAÇÕES ---
# Os modelos que você forneceu:
MODELOS = {
    "1m_pilar": 10,
    "2m_wall": 30,
    "5m_wall": 40,
    "10m_wall": 10
}

# Área de -25 a 25 metros em X e Y
X_BOUNDS = (-20.0, 20.0)
Y_BOUNDS = (-20.0, 20.0)
YAW_BOUNDS = (0.0, 2 * math.pi) # Rotação de 0 a 360 graus em radianos

# Número de cópias de cada modelo (total de 4 * 15 = 60 modelos)
COPIAS_POR_MODELO = 30
# ---------------------

output = []
model_counter = 0

output.append("")
output.append(f"\n")

for uri,numer in MODELOS.items():
    base_name = uri.replace('m_', '').replace('.', '_') # Nome base para o prefixo (ex: pilar_0_5)
    
    for i in range(1, COPIAS_POR_MODELO + 1):
        # Gera posições X e Y aleatórias
        x = round(random.uniform(*X_BOUNDS), 3)
        y = round(random.uniform(*Y_BOUNDS), 3)
        
        # Gera rotação Z aleatória (Yaw)
        yaw = round(random.uniform(*YAW_BOUNDS), 4)
        
        # Posição Z fixa no chão (0.0)
        z = 0.0 
        
        pose_str = f"{x} {y} {z} 0 0 {yaw}"
        unique_name = f"{base_name}_{i}"
        
        xml_block = (
            f"<include>\n"
            f"  <uri>model://{uri}</uri>\n"
            f"  <name>{unique_name}</name>\n"
            f"  <pose>{pose_str}</pose>\n"
            f"</include>\n"
        )
        output.append(xml_block)
        model_counter += 1

print('\n'.join(output))