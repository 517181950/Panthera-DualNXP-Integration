"""Historical absolute encoder-coordinate poses, in degrees, from 2026-10-29."""
import itertools


def linspace(start,end,n=7):
    return [start+(end-start)*i/(n-1) for i in range(n)]


def legacy285():
    poses=[]
    for joint,base,start,end in (
        (5,[0,90,180,-90,0,0],-135,135),
        (4,[0,90,180,0,0,0],-90,90),
        (3,[0,90,180,0,0,0],-90,90),
        (2,[0,90,0,0,0,0],200,45),
        (1,[0,0,135,0,0,0],30,160)):
        for q in linspace(start,end):
            point=base.copy();point[joint]=q;poses.append(point)
    for offsets in itertools.product((0,30,-30),repeat=5):
        poses.append([0]+[v+d for v,d in zip([90,90,0,0,0],offsets)])
    poses.extend([[0,q,10,30,0,90] for q in linspace(5,70)])
    return poses
