FROM osrf/ros:jazzy-desktop

# 1. Instala dependências do sistema e ferramentas
# Adicionamos 'python3-scipy' aqui para garantir que as dependências pesadas
# venham do APT (binários pré-compilados) e não precisem ser compiladas pelo PIP.
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    python3-scipy \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# 2. A CORREÇÃO MÁGICA:
# --break-system-packages: Permite instalar no sistema global (Ubuntu 24.04)
# --ignore-installed: Evita o erro "Cannot uninstall SciPy", forçando a instalação
#                     sem tentar deletar o pacote do sistema.
RUN pip3 install "numpy<2.0" cvxpy osqp --break-system-packages --ignore-installed
# 3. Configura o bash para carregar o ROS
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws

# Mantém o container aberto (compatível com o docker-compose que te passei antes)
CMD ["bash"]