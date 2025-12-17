#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# --- CONFIGURAZIONE MANUALE (SE NECESSARIO) ---
# Se non usi 'colcon build' o vuoi puntare ai file sorgente in src, 
# cambia questa variabile con il percorso assoluto:
# Esempio: "/home/tuonome/ros2_ws/src/ros2_fra2mo/maps/map.yaml"
CUSTOM_MAP_PATH = "" 

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.path_x = []
        self.path_y = []
        
        self.create_timer(0.1, self.record_pose)
        self.get_logger().info("Recorder avviato! Premi Ctrl+C per generare il grafico.")

    def record_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            if not self.path_x or abs(x - self.path_x[-1]) > 0.02 or abs(y - self.path_y[-1]) > 0.02:
                self.path_x.append(x)
                self.path_y.append(y)
        except Exception:
            pass

def main():
    rclpy.init()
    plotter = TrajectoryPlotter()
    
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        print("\nGenerazione Plot...")
        
        # --- CARICAMENTO MAPPA ROBUSTO ---
        img = None
        extent = None
        
        try:
            # 1. Trova il percorso del file YAML
            if CUSTOM_MAP_PATH:
                yaml_path = CUSTOM_MAP_PATH
            else:
                # Cerca nella cartella installata (share)
                pkg_path = get_package_share_directory('ros2_fra2mo')
                yaml_path = os.path.join(pkg_path, 'maps', 'map.yaml')
            
            print(f"Leggo mappa da: {yaml_path}")

            with open(yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
                
            resolution = map_data['resolution']
            origin = map_data['origin']
            image_filename = map_data['image']
            
            # 2. Costruisci il percorso ASSOLUTO dell'immagine
            # Prende la cartella dove sta il yaml e ci attacca il nome dell'immagine
            map_dir = os.path.dirname(yaml_path)
            img_full_path = os.path.join(map_dir, image_filename)
            
            print(f"Leggo immagine da: {img_full_path}")
            
            # 3. Carica e elabora l'immagine
            if not os.path.exists(img_full_path):
                print(f"ERRORE: Il file {img_full_path} non esiste!")
            else:
                img = mpimg.imread(img_full_path)
                img = np.flipud(img) # Correzione orientamento
                
                height, width = img.shape
                min_x = origin[0]
                max_x = origin[0] + (width * resolution)
                min_y = origin[1]
                max_y = origin[1] + (height * resolution)
                extent = [min_x, max_x, min_y, max_y]
                
        except Exception as e:
            print(f"ERRORE CRITICO caricamento mappa: {e}")
            print("Consiglio: Inserisci il percorso assoluto in CUSTOM_MAP_PATH all'inizio dello script.")

        # --- PLOTTING ---
        fig, ax = plt.subplots(figsize=(10, 8))
        
        if img is not None:
            ax.imshow(img, cmap='gray', extent=extent, origin='lower', alpha=0.5)
        
        ax.plot(plotter.path_x, plotter.path_y, '-b', linewidth=2, label='Traiettoria')
        if plotter.path_x:
            ax.plot(plotter.path_x[0], plotter.path_y[0], 'go', label='Start')
            ax.plot(plotter.path_x[-1], plotter.path_y[-1], 'ro', label='End')
        
        ax.set_title("Mission Trajectory")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        plt.savefig('mission_result_final.png', dpi=300)
        print("Salvato: mission_result_final.png")
        plt.show()

    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
