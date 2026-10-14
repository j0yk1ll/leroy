"""ChArUco PnP pose from the existing calibration board/detector."""
from pathlib import Path
import cv2
import numpy as np
from calibrate import load_board, detect_charuco

class CharucoPoseEstimator:
    def __init__(self,config,min_corners=8,max_rmse_px=2.0):
        self.config,self.dictionary,self.board=load_board(Path(config))
        self.xyz=np.asarray(self.board.getChessboardCorners(),dtype=np.float64)
        self.center=self.xyz.mean(axis=0)
        self.min_corners=min_corners
        self.max_rmse_px=max_rmse_px

    def pose(self,gray,info,_min_tags=None,_max_rmse=None):
        if info is None or gray is None or gray.shape[:2]!=(info.height,info.width):
            return None
        if info.distortion_model not in ('plumb_bob','rational_polynomial'):
            return None
        k=np.asarray(info.k,dtype=np.float64).reshape(3,3)
        d=np.asarray(info.d,dtype=np.float64)
        if k[0,0]<=0 or k[1,1]<=0 or not np.isfinite(k).all():
            return None
        corners,ids,_,_=detect_charuco(gray,self.board,self.dictionary)
        if ids is None or len(ids)<self.min_corners:
            return None
        ids=np.asarray(ids,dtype=np.int32).reshape(-1)
        if len(set(ids.tolist()))!=len(ids) or np.min(ids)<0 or np.max(ids)>=len(self.xyz):
            return None
        pix=np.asarray(corners,dtype=np.float64).reshape(-1,2)
        if np.ptp(pix[:,0])<40 or np.ptp(pix[:,1])<40:
            return None
        try:
            ok,rv,tv=cv2.solvePnP(self.xyz[ids],pix,k,d,flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:return None
            rot,_=cv2.Rodrigues(rv)
            center_c=rot@self.center+tv.reshape(3)
            if center_c[2]<=0:return None
            projected,_=cv2.projectPoints(self.xyz[ids],rv,tv,k,d)
            rmse=float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-pix)**2,axis=1))))
            if not np.isfinite(rmse) or rmse>self.max_rmse_px:return None
            uv,_=cv2.projectPoints(self.center[None,:],rv,tv,k,d)
            u,v=uv.reshape(2)
            return dict(rot=rot,location_b=-rot.T@tv.reshape(3),
                center_c=center_c,cx=float(u/info.width),cy=float(v/info.height),
                range_m=float(np.linalg.norm(center_c)),rmse=rmse,tags=len(ids))
        except cv2.error:
            return None
