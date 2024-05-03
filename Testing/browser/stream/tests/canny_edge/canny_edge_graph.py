# Include definitions from the Python package to
# define datatype for the IOs and to have access to the
# Graph class
from cmsis_stream.cg.scheduler import *

from ...appnodes import * 

# Define the datatype we are using for all the IOs in this
# example
# 

def mk_canny_edge(W=640,H=480,OPENCV=True):

    the_graph = Graph()
    the_graph.defaultFIFOClass  = ImageFIFO

    
    image_t = CImageType(W,H,t=CImageType.RGBA)
    
    camera=WebCamera("camera",W,H)
    
    
    # Canny edge
    to_gray8 = RGBAToGray8("to_gray8",W,H)
    gaussian = GaussianFilter("gaussian",W,H)
    
    
    canny_edge = CannyEdge("canny",W,H,config=True)
    to_rgba = Gray16ToRGBA("to_rgba",W,H)
    
    display1=WebDisplay1("display1",W,H)

    if OPENCV:
       g16_to_g8 = Gray16ToGray8("g16_to_g8",W,H)
       canny_edge_cv = OpenCVCanny("canny_cv",W,H,firstAlgo=False,config=True)
       cv_to_rgba = Gray8ToRGBA("cv_to_rgba",W,H)
       display2=WebDisplay2("display2",W,H)
        
    gauss = gaussian(to_gray8(camera(the_graph)))
    
    cv = canny_edge(gauss)
    
    display1(to_rgba(cv))

    if OPENCV:
       ocv = canny_edge_cv(g16_to_g8(gauss))
       display2(cv_to_rgba(ocv))

    return(the_graph)


