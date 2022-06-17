import numpy as np
import os
import pymeshlab
import pdb 
import open3d as o3d

#townid=3
#dir_='/mnt/workspace/users/askc/ILLUMINATION-TRANSFER-PTCLOUD-RGB/datasets/CARLA-SUN-SYNC/Town0%d_Opt/episode0/'%townid
dir_='/mnt/workspace/users/askc/ILLUMINATION-TRANSFER-PTCLOUD-RGB/datasets/CARLA-SUN-SYNC/Town03_Opt/episode0/sunloc_40.0,36.66666666666667/colmap_format/dense'
filename = 'meshed-delaunay.ply'
meshpath=os.path.join(dir_,filename)
ms = pymeshlab.MeshSet()
ms.load_new_mesh(meshpath)
ms.apply_filter('ambient_occlusion')
ms.save_current_mesh(dir_+'/' + os.path.splitext(filename)[0]+'_ambient_occlusion_' + str(128) + '.ply')
#pdb.set_trace()

## load in ambient occlusion mesh
mesh = o3d.io.read_triangle_mesh(dir_+'/' + os.path.splitext(filename)[0]+'_ambient_occlusion_' + str(128) + '.ply')
verts = np.asarray(mesh.vertices)
verts_vis = np.asarray(mesh.vertex_colors)[:,0]

pdb.set_trace()

## load in poses
from load_llff_hotdogblender import *

img_id = 0
#datadir = '/mnt/workspace/users/askc/ILLUMINATION-TRANSFER-PTCLOUD-RGB/datasets/Nerfactor_dataset/hotdog_2163/colmap_format_singleillum_city'
datadir = '/root/dataset_temp'
start_frame = 0
end_frame = 50
factor = 8
target_idx = 10
final_height=288
## load in dataset
images, depths, normals, masks, albedos, roughs, envmaps, poses, bds, render_poses, ref_c2w, motion_coords, sky_coords, nonsky_coords, sunpos_xyz, trainfilenames = load_llff_data(datadir,
                                                    start_frame, end_frame, factor,
                                                    target_idx=target_idx,
                                                    recenter=True, bd_factor=.9,
                                                    spherify=False,
                                                    final_height=final_height)

depth_img = torch.Tensor(depths[img_id, :,:])
rgbflat = images[img_id,:,:,:].reshape(-1,3)
c2w = torch.Tensor(poses[img_id, :,:])
#pdb.set_trace()
w2c = torch.cat([c2w[:,:3].T, torch.mm(-c2w[:,:3].T, c2w[:,3,None])], dim=-1)
focal = 192.67474
