#!/usr/bin/env python

'''
created by https://github.com/ylchan87/OpticalFlow_Test
modified by me (yueqianliu@outlook.com)

Sample to show optical flow estimation using various algos

Modified from:
official openCV samples
https://github.com/opencv/opencv/blob/master/samples/python/dis_opt_flow.py


USAGE: python opt_flow_test.py [<video_source>] [<mode>]
where
    <video_source>  : filename, or 0 for Camera0, 1 for Camera1 etc.  (default:0)
    <mode>          : 'testAll' to run all algo and save (default:'interact')

With 'interactive' mode, controls at pop up wndow are:
 t   - toggle temporal propagation of flow vectors
 1-8 - switch to use different algo 
 ESC - exit


Examples:

  # Run and interact with camera0 (usually the webcam at notebook)
  python opt_flow_test.py

  # Use a file as source and interact
  python opt_flow_test.py input.mp4

  # Run all algo on the file and save result as video
  python opt_flow_test.py input.mp4 testAll

'''

import numpy as np
import cv2 as cv
import time
import os.path

def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
    fx, fy = -flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    if len(img.shape)==2:
        vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    else:
        vis = img.copy()
    cv.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (_x2, _y2) in lines:
        cv.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis


def draw_hsv(flow, bkgBlack = True):
    h, w = flow.shape[:2]
    fx, fy = flow[:,:,0], flow[:,:,1]
    ang = np.arctan2(fy, fx) + np.pi
    v = np.sqrt(fx*fx+fy*fy)
    hsv = np.zeros((h, w, 3), np.uint8)
    hsv[...,0] = ang*(180/np.pi/2)
    if bkgBlack:
        hsv[...,1] = 255
        hsv[...,2] = np.minimum(v*4, 255)
    else:
        hsv[...,1] = np.minimum(v*4, 255)
        hsv[...,2] = 255
    bgr = cv.cvtColor(hsv, cv.COLOR_HSV2BGR)
    return bgr


def warp_flow(img, flow):
    h, w = flow.shape[:2]
    flow = -flow
    flow[:,:,0] += np.arange(w)
    flow[:,:,1] += np.arange(h)[:,np.newaxis]
    res = cv.remap(img, flow, None, cv.INTER_LINEAR)
    return res


class OptFlowTester:
    
    algs = [
        "Dummy",
        "DIS_Medium",
        "DIS_Fast",
        "DIS_UltraFast",
        "Farneback",
        "DenseRLOF",
        "DualTVL1",
        "PCAFlow",
        "DeepFlow",
    ]
    
    def setAlg(self, opt):
        if type(opt) == int and 0<=opt<len(OptFlowTester.algs):
            opt = OptFlowTester.algs[opt]
        self.algName = opt
        

        self.useGrayInput = True

        if opt=="DIS_Medium":
            self.alg = cv.DISOpticalFlow.create(cv.DISOPTICAL_FLOW_PRESET_MEDIUM)
        elif opt=="DIS_Fast":
            self.alg = cv.DISOpticalFlow.create(cv.DISOPTICAL_FLOW_PRESET_FAST)
        elif opt=="DIS_UltraFast":
            self.alg = cv.DISOpticalFlow.create(cv.DISOPTICAL_FLOW_PRESET_ULTRAFAST)
        elif opt=="Farneback":
            self.alg = cv.FarnebackOpticalFlow_create()
        elif opt=="DenseRLOF":
            self.alg = cv.optflow.DenseRLOFOpticalFlow_create()
            self.useGrayInput = False
        elif opt=="DualTVL1":
            self.alg = cv.optflow.DualTVL1OpticalFlow_create()
        elif opt=="PCAFlow":
            self.alg = cv.optflow.createOptFlow_PCAFlow()
        elif opt=="DeepFlow":
            self.alg = cv.optflow.createOptFlow_DeepFlow()
        else:
            self.alg = None
            self.algName = None
            print("Unknown alg {opt}")

    def run(self, output = None):

        if self.alg is None: 
            print ("Algo not yet set")
            return False
        
        if self.input is None:
            print ("Input source not set")
            return False

        if self.cam: self.cam.release()
        self.cam = cv.VideoCapture(self.input)    

        if output:
            fourcc = cv.VideoWriter_fourcc(*'XVID')
            fps = self.cam.get(cv.CAP_PROP_FPS)
            h = int(self.cam.get(cv.CAP_PROP_FRAME_HEIGHT))
            w = int(self.cam.get(cv.CAP_PROP_FRAME_WIDTH))
            output = cv.VideoWriter(output, fourcc, fps, (w*2, h), True)

        firstFrame = True

        while True:
            _ret, img = self.cam.read()
            if not _ret: break
            inputImg = cv.cvtColor(img, cv.COLOR_BGR2GRAY) if self.useGrayInput else img

            if firstFrame:
                h,w = inputImg.shape[0:2]
                flow = np.zeros( (h,w,2) , dtype=np.float32)
                prevInputImg = inputImg
                firstFrame = False
                continue

            time0 = time.time()
            
            guess = None
            if self.use_temporal_propagation: 
                #warp previous flow to get an initial approximation for the current flow
                guess = warp_flow(flow,flow)

            try:
                flow = self.alg.calc(prevInputImg, inputImg, guess)
            except Exception as e:
                print(e)
                flow[:] = 0.0

            prevInputImg = inputImg

            time1 = time.time()        
            fps = 1 / (time1 - time0)
            
            visArrow = draw_flow(img, flow)
            visHSV = draw_hsv(flow, bkgBlack=True)
            vis = np.concatenate([visArrow, visHSV], axis=1)
            txtMsg = f"ALG: {self.algName}, FPS: {round(fps,2)}"
            cv.putText(vis, txtMsg, (10, 70), cv.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
            cv.imshow('flow', vis)

            ch = 0xFF & cv.waitKey(5)
            if ch == 27: break

            if self.interactive:
                if ch == ord('t'):
                    self.use_temporal_propagation = not self.use_temporal_propagation
                    print('temporal propagation is', ['off', 'on'][self.use_temporal_propagation])
                if 0 <= ch-ord('0') <=9 :
                    opt = ch-ord('0')
                    self.setAlg(opt)
            
            if output: output.write(vis)

        if output: output.release()
        self.cam.release()
        self.cam = None

        print('Done')
        return True

    def __init__(self) -> None:
        
        self.input = None
        self.cam = None
        self.alg = None
        self.algName = ""

        self.interactive = True
        self.useGrayInput = True    
        self.use_temporal_propagation = True # provide last flow as init guess to alg

def main():
    import sys
    print(__doc__)

    try:
        fn = sys.argv[1]
        if fn.isdigit(): fn = int(fn)
    except IndexError:
        fn = 0

    try:
        mode = sys.argv[2]
    except:
        mode = "interact"
    
    algIdx = 1
    if mode.isdigit(): algIdx = int(mode)

    optFlowTester = OptFlowTester()
    optFlowTester.input = fn
    optFlowTester.setAlg(algIdx)

    if mode=="interact":
        optFlowTester.run()
    if mode=="testAll" and type(fn)==str:
        optFlowTester.interactive = False
        for algName in OptFlowTester.algs[1:]:
            print(f"Running {algName}")
            optFlowTester.setAlg(algName)
            dirName, fileName = os.path.split(fn)
            fileName, _ = os.path.splitext(fileName)
            outName = os.path.join(dirName, fileName + f"_{algName}.mp4")
            optFlowTester.run(outName)
    
if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()