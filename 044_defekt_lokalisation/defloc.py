import glob
import cv2
import pandas
import defloc
import numpy
import pickle
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def get_dataset(dataset_dir):
    
    # 1. Daten einlesen
    liste_bilder = glob.glob( f"{dataset_dir}/*.png" )
    liste_bilder.sort()
    imgs = [cv2.imread(fname, cv2.IMREAD_GRAYSCALE) for fname in liste_bilder]
    t = pandas.read_csv( f"{dataset_dir}/defboxes.csv", index_col="Unnamed: 0" )
    imgs = []
    defboxes = []
    for fname in liste_bilder:
        img = cv2.imread(fname, cv2.IMREAD_GRAYSCALE)
        imgs.append( img )

        s = t.loc[fname]
        defbox = (s["x0"], s["y0"], s["w"], s["h"])
        defboxes.append( defbox )
        
    # 2. Train-Test-Split
    N = len(imgs)
    train_N = int(0.8 * N)
    imgs_train     = imgs[:train_N]
    defboxes_train = defboxes[:train_N]
    imgs_test      = imgs[train_N:]
    defboxes_test  = defboxes[train_N:]

    return imgs_train, defboxes_train, imgs_test, defboxes_test    


def sliding_window(img, winsize, step=None):
    
    if step==None:
        step=winsize
    
    img_height, img_width = img.shape
    
    ROIs = []
    for y in range(0, img_height - winsize - 1, step):
        for x in range(0, img_width - winsize - 1, step):
            ROI = (x,y,winsize,winsize)
            ROIs.append(ROI)
            
    return ROIs



def get_ground_truth_label(defbox, ROI, show=False):
    
    dx,dy,dw,dh = defbox
    rx,ry,rw,rh = ROI
    
    from shapely.geometry import Polygon
    p1 = Polygon( [(dx,dy), (dx+dw,dy), (dx+dw,dy+dh), (dx,dy+dh)] )
    p2 = Polygon( [(rx,ry), (rx+rw,ry), (rx+rw,ry+rh), (rx,ry+rh)] )
    
    if show:
        plt.xlim(min(dx,rx)-5,max(dx+dw,rx+rw)+5)
        plt.ylim(min(dy,ry)-5,max(dy+dh,ry+rh)+5)
        
        plt.gca().add_patch(Rectangle((dx,dy),dw,dh,
                                      edgecolor='blue',
                                      facecolor='none',
                                      lw=1))
        plt.gca().add_patch(Rectangle((rx,ry),rw,rh,
                                      edgecolor='green',
                                      facecolor='none',
                                      lw=1))
        plt.show()
        
    intersect = p1.intersects(p2)
    label = int(intersect)
    return label



def get_descriptor_vector(img, ROI, show=False):
    
    
    def map_img_ROI_to_descriptor_vector_VERSION_RAW(img_ROI):
        vec = img_ROI.flatten()
        return vec
    
    x0,y0,w,h = ROI
    img_ROI = img[y0:y0+h, x0:x0+w]
    
    if show:
        plt.imshow(img_ROI, cmap="coolwarm", vmin=50, vmax=160)
        plt.colorbar()
        
        plt.show()
    
    vec = map_img_ROI_to_descriptor_vector_VERSION_RAW(img_ROI)
    return vec



def get_vecs_and_labels(imgs, defboxes):

    N = len(imgs)

    vecs = []
    labels = []

    for id in range(0,N):

        # get image
        img = imgs[id]

        # compute ROIs for that image
        ROIs = sliding_window(img, 20)

        # get defect box for that image
        defbox = defboxes[id]

        # for each ROI compute
        # - descriptor vector and
        # - class label
        # and store both
        for ROI in ROIs:

            # compute descriptor vector
            vec = get_descriptor_vector(img, ROI)

            # compute classification label
            x0,y0,w,h = ROI
            label = get_ground_truth_label(defbox, ROI)

            vecs.append( vec )
            labels.append( label )
            
    return vecs, labels


def list_of_vecs_to_arrays(vecs, labels):
    print( f"Len of list of vecs  : {len(vecs)} ")
    print( f"Len of list of labels: {len(labels)} ")
    
    print( f"Len of one vec: {len(vecs[0])} ")
    
    nr_rows = len(vecs)
    nr_cols = len(vecs[0])
    
    x = numpy.zeros((nr_rows, nr_cols))
    y = numpy.zeros((nr_rows, 1))
    
    for i in range(len(vecs)):
        x[i] = numpy.array( vecs[i] )
        y[i] = labels[i]
        
    return x,y


def show_img_defbox_rois_and_classify(title_text, img, defbox, ROIs, model):
    
    plt.figure( figsize=(20,15) )
    
    plt.imshow(img, cmap="coolwarm", vmin=50, vmax=160)
    plt.colorbar()
    
    for i, ROI in enumerate(ROIs):
        x0,y0,w,h = ROI
        label = get_ground_truth_label(defbox, ROI)
            
        # classify ROI according to model
        # is it a defect or not?
        vec = get_descriptor_vector(img, ROI)
        pred = int(model.predict( [vec] )[0])
        print(pred, end=" ")
        
        # show prediction
        # red: defect
        # green: ok
        if pred==0:
            fcolor="none"
        elif pred==1:
            fcolor="red"
        plt.gca().add_patch(Rectangle((x0,y0),w,h,
                                  edgecolor='gray',
                                  facecolor=fcolor,
                                  lw=1, alpha=0.2))
    
    
    # show defect box with the help of a rectangle
    def_x0, def_y0, def_width, def_height = defbox
    plt.gca().add_patch(Rectangle((def_x0,def_y0),def_width,def_height,
                                  edgecolor='black',
                                  facecolor='none',    
                                  lw=2))
    
    plt.title(title_text)
    plt.show()