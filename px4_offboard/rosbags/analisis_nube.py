import open3d as o3d
import numpy as np
import warnings
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores  # Importamos 'Stores'

# Ignorar avisos de versiones de numpy/scipy
warnings.filterwarnings("ignore", category=UserWarning)

def analizar_nube(bag_path):
    puntos = []
    
    # CORRECCIÓN AQUÍ: Especificamos que queremos el almacén de mensajes de ROS 2
    typestore = get_typestore(Stores.ROS2_FOXY) # Probamos con FOXY que es el de tu dron
    
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == '/tof_pc']
        if not connections:
            print(f"No se encontró el tópico /tof_pc en {bag_path}")
            return

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            
            # Procesamiento de la nube (VOXL TOF data)
            data = np.frombuffer(msg.data, dtype=np.float32)
            step = msg.point_step // 4
            data = data.reshape(-1, step)
            puntos = data[:, :3] 
            print(f"Cargados {len(puntos)} puntos del primer frame.")
            break 

    if len(puntos) == 0:
        print("No se pudieron extraer puntos.")
        return

    # Limpiar NaN e Infinitos
    puntos = puntos[np.all(np.isfinite(puntos), axis=1)]

    # --- INVESTIGACIÓN RANSAC ---
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(puntos)

    # Segmentación del plano
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.03,
                                             ransac_n=3,
                                             num_iterations=1000)
    
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    ratio_plano = len(inliers) / len(puntos)
    print(f"\n--- INVESTIGACIÓN DE SUPERFICIE ---")
    print(f"Ecuación: {plane_model[0]:.2f}x + {plane_model[1]:.2f}y + {plane_model[2]:.2f}z + {plane_model[3]:.2f} = 0")
    print(f"Porcentaje de planitud: {ratio_plano*100:.2f}%")

    if ratio_plano > 0.75:
        print("CONCLUSIÓN: Es una superficie PLANA.")
    else:
        print("CONCLUSIÓN: OBJETO DETECTADO (Superficie no plana).")

    # Visualización
    inlier_cloud.paint_uniform_color([0, 1, 0]) 
    outlier_cloud.paint_uniform_color([1, 0, 0]) 
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

if __name__ == "__main__":
    analizar_nube('solo_un_frame')
