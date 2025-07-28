## 3D Understanding of Deformable Linear Objects: Datasets and Transferability Benchmark (WACV 2025)

[Bare Luka Zagar](https://scholar.google.com/citations?hl=zh-CN&user=QEz7ItgAAAAJ), [Mingyu Liu](https://scholar.google.com/citations?user=Bcohc-oAAAAJ), [Tim Hertel](https://de.linkedin.com/in/tim-hertel), [Ekim Yurtsever](https://scholar.google.com/citations?hl=zh-CN&user=dJGmJCEAAAAJ), and [Alois C. Knoll](https://scholar.google.com/citations?hl=zh-CN&user=-CA8QgwAAAAJ&view_op=list_works)

>**Abstract:** Deformable linear objects are commonly found in our daily lives. Understanding them visually can be challenging even for humans, as the same object can become entangled and look completely different. Examples of deformable linear objects include blood vessels and wiring harnesses, which are crucial for the proper functioning of systems like the human body and vehicles. Recently, some studies have focused on 2D image segmentation of wires. However, there are no point cloud datasets available for studying 3D deformable linear objects, which are more complex and challenging. To address this gap, we introduce two point cloud datasets, PointWire and PointVessel, generated using our proposed semi-automatic pipeline. We evaluated state-of-the-art methods on these large-scale 3D deformable linear object benchmarks. Additionally, we analyzed the generalization capabilities of these methods through transferability experiments on the PointWire and PointVessel datasets.

---
### Deformable Linear Objects
<img src="/figures/DLO_figure.png">

### Data Generation Pipeline
<img src="/figures/Data_generation.png">

1. For the cleaning of the original pointcloud, we provide the file `dataset_generation/pcl_preprocessing.py`.
   This assumes that there is a plane on which the object is presented. It fits the plane to it and removes the points of the plane.
2. We fit a basic armature to it with `dataset_generation/laplace_skeleton.py`. This will create a .npz file with the nodes and adjacency matrix.
3. The clean pointclouds and skeletons should be stored in folders `clean` and `skeleton`. Then, you can automatically import them into blender with the    
   `dataset_generation/create_armature_from_graph.py` script. Further instructions on the usage can be found in the script.
4. Now, the data needs to be manually edited with blender. We first improved the armature and moved it to represent the wire harness as good as possible.
   We also grouped the points that should be assigned to noise in a `none` group and the nodes that belong to a connector in a `connector` group.
5. To animate it more easily, we used the `generate_animation_control` script, which automatically sets up inverse kinematics.
6. The pointclouds, the segmentation based on the thickness of the wire and the skeleton can then be exported with the scripts `export_pcls.py`, 
   `export_segmentation.py` and `export_skeleton.py`.
7. If you need equally sized samples, you can run the fps algorithm on the pointcloud and use the indices on the segmentation as well.


### CDLO Datasets

|Dataset | Link |
| :----- | :--: |
|Pointwire | [Nextcloud](https://nextcloud.in.tum.de/index.php/s/7ooyYxoP6HyPXQK) |
|Pointvessel | [Nextcloud](https://nextcloud.in.tum.de/index.php/s/7ooyYxoP6HyPXQK) |
