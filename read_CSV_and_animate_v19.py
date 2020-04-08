import csv
import hou
import numpy as np

cutoff = 1000000 #for testing - make it shorter to test fewer frames
trackmodecsv = '/path/to/sample_csv/telemetry-v1-2020-03-14-14_38_35.csv'

def sumforline(filename):##USED TO COUNT TOTAL LINES
    with open(filename) as f:
        return sum(1 for line in f)

def returnbesttime(sequence):
    low = sequence[0] # need to start with some value
    for i in sequence:
        if i < low:
            low = i
    return low

def returnlowesthighest(sequence):
    low = sequence[0] # need to start with some value
    high = sequence[0] # need to start with some value
    for i in sequence:
        if i < low:
            low = i
        if i > high:
            high = i
    return low,high

def returnmaxspeed(sequence):
    high = sequence[0]
    for i in sequence:
        if i > high:
            high = i
    return high

def returnclocktime(time):
    hours = 0
    minutes = 0
    miliseconds = time
    hours, milliseconds = divmod(miliseconds, 3600000)
    minutes, milliseconds = divmod(miliseconds, 60000)
    seconds = float(milliseconds) / 1000
    s = "%02i:%05.2f" % (minutes, seconds)
    return s

def clamp(num):
    if num > 100.0:
        num = 100.0
    if num < 0.01:
        num = 0.0
    return num

totalframes = int(sumforline(trackmodecsv))-1

laplist = list()#index 0
timelist = list()#index 1
speedlist = list()#index 2
latlist = list()#index 3
longlist = list()#index 4
lataccellist = list()
longaccellist = list()
throttlelist = list()#index 7
brakelist = list()#index 8
steeranglelist = list()#index 9
soclist = list()#index 13
braketempfrontleftlist = list()#index 18
braketempfrontrightlist = list()#index 19
braketemprearleftlist = list()#index 20
braketemprearrightlist = list()#index 21
batterytemplist = list()#index 24
tireslipfrontleftlist = list()#index 25
tireslipfrontrightlist = list()#index 26
tireslipbackleftlist = list()#index 27
tireslipbackrightlist = list()#index 28
lapstartdict = dict()

##CALCULATE LATER
linecounter = 0
with open(trackmodecsv) as csvfile:
    csvreader = csv.reader(csvfile, delimiter=',')
    for lines in csvreader:
        if linecounter == 0:##PARSE HEADERS
            speedstandard = lines[2].replace('Speed (','').replace(')','')
        if linecounter > 0:##skip the text header
            laplist.append(int(lines[0]))
            timelist.append(lines[1])
            speedlist.append(int(float(lines[2])))
            latlist.append(float(lines[3]))##Y AXIS
            longlist.append(float(lines[4]))##X AXIS
            lataccellist.append(float(lines[5]))##Y AXIS
            longaccellist.append(float(lines[6]))##X AXIS
            throttlelist.append(float(lines[7]))
            brakelist.append(float(lines[8]))
            steeranglelist.append(float(lines[9]))
            soclist.append(float(lines[13]))
            braketempfrontleftlist.append(float(lines[18]))
            braketempfrontrightlist.append(float(lines[19]))
            braketemprearleftlist.append(float(lines[20]))
            braketemprearrightlist.append(float(lines[21]))
            batterytemplist.append(lines[24])
            tireslipfrontleftlist.append(float(lines[25]))
            tireslipfrontrightlist.append(float(lines[26]))
            tireslipbackleftlist.append(float(lines[27]))
            tireslipbackrightlist.append(float(lines[28]))
        linecounter+=1 


##DELETE PREVIOUS KEYFRAMES TO BE SAFE OR STAGGERED KEYFRAMES WILL FAIL:
node = hou.node("/obj/track/current_car_pos") 
parm_tuple = node.parmTuple("t") 
parm_tuple.deleteAllKeyframes()

node = hou.node("/obj/track/lap_data") 
parm = node.parm("value1v1") 
parm.deleteAllKeyframes()

node = hou.node("/obj/speedometer/speed_holder") 
parm = node.parm("value1v1") 
parm.deleteAllKeyframes()

node = hou.node("/obj/lap_run_time/switch1") 
parm = node.parm("input") 
parm.deleteAllKeyframes()

#STEERING WHEEL 
node = hou.node('/obj/steering_wheel/driven_transform')
parm = node.parm("rz")
parm.deleteAllKeyframes()

#SOC METER
node = hou.node('/obj/soc_meter/attribcreate_soc')
parm = node.parm("value1v1")
parm.deleteAllKeyframes()

#BRAKE PRESSURE METER
node = hou.node('/obj/brakes/attribcreate_brake')
parm = node.parm("value1v1")
parm.deleteAllKeyframes()

#THROTTLE METER
node = hou.node('/obj/throttle/attribcreate_throttle')
parm = node.parm("value1v1")
parm.deleteAllKeyframes()

##BRAKE PEDAL
node = hou.node('/obj/brakes/attribcreate_brake')
parm = node.parm("value1v1")
parm.deleteAllKeyframes()

##BRAKE TEMPS
node = hou.node('/obj/model_3_tires_rear_fbx/rear_left_tire/disk_temp_att')
parm = node.parm("value1v1")
parm.deleteAllKeyframes()

node = hou.node('/obj/model3_tire_front_left_tire_new/disk_temp_att')
parm = node.parm("value1v1")
parm.deleteAllKeyframes()

##TRACTION
for nodestring in ['attribcreate_slip_frontleft','attribcreate_slip_frontright','attribcreate_slip_backright','attribcreate_slip_backleft']:
    node = hou.node('/obj/traction/' + nodestring)
    parm = node.parm("value1v1")
    parm.deleteAllKeyframes()

##GMETER
node = hou.node('/obj/gforce/latlongaccel')
parm = node.parm("value1v1")
parm.deleteAllKeyframes()
parm = node.parm("value2v1")
parm.deleteAllKeyframes()

##END OF KEYFRAME DELETES


##ONE-OFF STUFF THAT DOESN'T NEED TO HAPPEN IN THE FRAME LOOP###################
fullrotationatframe = list()##this is what is used for transform node entries
factor = 1.0
currentlap = 0
laptimes = list()
lapframes = list()##FOR LAP SPEED MARKERS
laptimesclocklist = list()

for x in range(0, totalframes):
    ##NORMALIZE BRAKE PRESSURE FROM VALUES. ANYTHING OVER 60 IS MAX
    brakelist[x] = clamp(brakelist[x])

##MAKE ENTRIES FOR LAP DICT

    if x == 0:
        fullrotationatframe.append(0.0)
    else:
        fullrotationatframe.append(speedlist[x]*factor + fullrotationatframe[x-1])

##LAP TIMER STUFF
    if laplist[x] > currentlap:##WE HAVE HIT THE NEXT LAP SO WRITE OUT THE LAP TIME AND BUMP LAP COUNTER
        laptimes.append(int(timelist[x-1]))
        lapframes.append(x)
        currentlap += 1
    elif x == totalframes-1:##REACHED LAST ENTRY
        laptimes.append(int(timelist[x]))
        lapframes.append(x)
for time in laptimes:
    s = returnclocktime(time)
    laptimesclocklist.append(s)

##SET LAP TIME STRINGS IN /obj/lap_run_time/fontx - doens't need to be done every frame 
for m in range(0, len(laptimesclocklist)):
    lapattrnode = hou.node('/obj/lap_run_time/lap' + str(m))
    laptimestring = str(laptimesclocklist[m])
    lapattrnode.parm('string1').set(laptimestring)

##SET BEST LAP TIME STRING
lapattrnode = hou.node('/obj/best_lap/lap0')
bestlaptime = returnbesttime(laptimes)
bestlaptimeclock = returnclocktime(bestlaptime)
lapattrnode.parm('string1').set(str(bestlaptimeclock))

##SET KMPH OR MPH
speedholder = hou.node('/obj/speedometer/speed_holder')
speedholder.parm('string2').set(speedstandard)

##DERIVE WHICH IS THE LONGEST SIDE AND SET THE MATCH SIZE AXIS ACCORDINGLY
lowesthighestxvals = returnlowesthighest(longlist)
lowesthighestyvals = returnlowesthighest(latlist)

if lowesthighestxvals[1]-lowesthighestxvals[0] > lowesthighestyvals[1]-lowesthighestyvals[0]:
    hou.node('/obj/track/matchsize1').parm('scale_axis').set(0)
else:
    hou.node('/obj/track/matchsize1').parm('scale_axis').set(1)

##SET THE BOUNDARY POINTS FOR BOTTOM LEFT
hou.node('/obj/track/boundary').parm('pt0x').set(lowesthighestxvals[0])
hou.node('/obj/track/boundary').parm('pt0y').set(lowesthighestyvals[0])

##SET THE BOUNDARY POINTS FOR TOP RIGHT OF CURRENTLY DRAWING LAP
hou.node('/obj/track/boundary').parm('pt1x').set(lowesthighestxvals[1])
hou.node('/obj/track/boundary').parm('pt1y').set(lowesthighestyvals[1])

##SET LAP COLOR RANGE FROM TOTAL LAPS
lapcolornode = hou.node('/obj/track/lap_color')
lapcolornode.parm('ramprange2').set(laplist[-1])

##SET SPEED COLOR RANGE FROM MAX SPEED
maxspeed = returnmaxspeed(speedlist)
speedcolornode = hou.node('/obj/track/speed_color')
speedcolornode.parm('ramprange2').set(maxspeed)


##PEAK AND VALLEY DETECTION - This method fails if there are subsequent duplicate entries in the array so you have to work on a filtered version with no dupe entries
##https://tcoil.info/find-peaks-and-valleys-in-dataset-with-python/
values = list()
prevval = None
for x in range(0, totalframes):
    if speedlist[x] != prevval:
        values.append(speedlist[x])
    prevval = speedlist[x]

data = np.array(values)
#     ___ detection of local minimums and maximums ___
a = np.diff(np.sign(np.diff(data))).nonzero()[0] + 1               # local min & max
b = (np.diff(np.sign(np.diff(data))) > 0).nonzero()[0] + 1         # local min
c = (np.diff(np.sign(np.diff(data))) < 0).nonzero()[0] + 1         # local max
# +1 due to the fact that diff reduces the original index number

minmaxspeedlist = data[a].tolist()
minmaxspeedlistpared = list()
threshold = 8##PARE DELTAS SMALLER OR GREATER THAN THIS
for x in range(0, len(minmaxspeedlist)):##FILTER SMALL DELTAS SINCE WE DON'T CARE ABOUT THEM 
    if x == 0:
        minmaxspeedlistpared.append(minmaxspeedlist[0])
    if x > 0 and not x == len(minmaxspeedlist)-1:
        if abs(minmaxspeedlist[x])+threshold < minmaxspeedlist[x+1] or abs(minmaxspeedlist[x])-threshold > minmaxspeedlist[x+1]:
            minmaxspeedlistpared.append(minmaxspeedlist[x])
    if x == len(minmaxspeedlist)-1:
         minmaxspeedlistpared.append(minmaxspeedlist[x])



##START OF GENERAL FRAME LOOPING###################
minmaxspeedindexpoint = 0##TO TICK UP ON MATCH 
previouslap = 0
previousspeed = 0
previousbrakepressure = 0.0
previousthrottle = 0.0
alreadysetbitflip = 0

for x in range(0, totalframes):
    if x < cutoff:
        hou.setFrame(x)#STEP TO FRAME

        ##STEERING WHEEL
        if x%30 == 0 or x == 0 or x == totalframes-1:##EVERY NTH FRAME SINCE IT'S OVERKILL TO KEYFRAME POSITION BY MILLISECOND
            steervalatframe = steeranglelist[x]
            wheeltransform = hou.node('/obj/steering_wheel/driven_transform')
            key = hou.Keyframe(x)
            wheeltransform.parm("rz").setKeyframe(key)
            wheeltransform.parm('rz').set(steervalatframe*-1)##flipped

        ##BRAKE PEDAL 
        if not x==totalframes-1:
            if brakelist[x] != previousbrakepressure or brakelist[x+1] != previousbrakepressure:##Don't keyframe before the change since we want interpolation between frames
            # if x == 0 or x%5==0:##don't keyframe a ton since brake are mostly not used
                brakeattributeholder = hou.node('/obj/brakes/attribcreate_brake')
                key = hou.Keyframe(x)
                brakeattributeholder.parm('value1v1').setKeyframe(key)
                brakeattributeholder.parm('value1v1').set(brakelist[x])
        
        previousbrakepressure = brakelist[x]

        ##BRAKE TEMPS
        backleftbrake = hou.node('/obj/brake_temps/back_left/blend1')
        if x%200==0 or x==0 or x==totalframes-1:##EVERY NTH FRAME SINCE IT'S OVERKILL TO KEYFRAME POSITION BY X MILLISECOND
            key = hou.Keyframe(x)
            node = hou.node('/obj/model_3_tires_rear_fbx/rear_left_tire/disk_temp_att')
            node.parm('value1v1').setKeyframe(key)
            node.parm('value1v1').set(braketemprearleftlist[x])

            node = hou.node('/obj/model3_tire_front_left_tire_new/disk_temp_att')
            node.parm('value1v1').setKeyframe(key)
            node.parm('value1v1').set(braketempfrontleftlist[x])

        ##THROTTLE
        if not x==totalframes-1:
            if x%5==0:##don't keyframe a ton since we are going to do interpolation between frames anyway
                throttenode = hou.node('/obj/throttle/attribcreate_throttle')
                key = hou.Keyframe(x)
                throttenode.parm('value1v1').setKeyframe(key)
                throttenode.parm('value1v1').set(throttlelist[x])
        
        previousthrottle = throttlelist[x]


        ##TIRE TRACTION
        if x == 0 or x%5==0:##don't keyframe a ton since we are going to do interpolation between frames anyway
            node = hou.node('/obj/traction/attribcreate_slip_backleft')
            key = hou.Keyframe(x)
            node.parm('value1v1').setKeyframe(key)
            node.parm('value1v1').set(abs(tireslipbackleftlist[x]))
            
            node = hou.node('/obj/traction/attribcreate_slip_backright')
            key = hou.Keyframe(x)
            node.parm('value1v1').setKeyframe(key)
            node.parm('value1v1').set(abs(tireslipbackrightlist[x]))
            
            node = hou.node('/obj/traction/attribcreate_slip_frontright')
            key = hou.Keyframe(x)
            node.parm('value1v1').setKeyframe(key)
            node.parm('value1v1').set(abs(tireslipfrontrightlist[x]))

            node = hou.node('/obj/traction/attribcreate_slip_frontleft')
            key = hou.Keyframe(x)
            node.parm('value1v1').setKeyframe(key)
            node.parm('value1v1').set(abs(tireslipfrontleftlist[x]))


        ##SOC METER
        if x == 0:
            socstartval = soclist[0]##the fixed size for the box that shows where you started with your SOC before the laps started
            socstartgeo = hou.node('/obj/soc_meter/initialsoctrans')
            socstartgeo.parm('sy').set(soclist[x]/100)

        if x%150==0 or x==totalframes-1:##EVERY NTH FRAME SINCE IT'S OVERKILL TO KEYFRAME SOC BY MILLISECOND
            socattributeholder = hou.node('/obj/soc_meter/attribcreate_soc')
            key = hou.Keyframe(x)
            socattributeholder.parm('value1v1').setKeyframe(key)
            socattributeholder.parm('value1v1').set(soclist[x]/100)


        ##GENERATE TRACK POINTS FROM GPS DATA AND WRITE TO TRANSFORM
        carpositionnode = hou.node('/obj/track/current_car_pos')
        if x%50==0 or x==0 or x==totalframes-1:##EVERY NTH FRAME SINCE IT'S OVERKILL TO KEYFRAME POSITION BY MILLISECOND
            key = hou.Keyframe(x)
            carpositionnode.parm('tx').setKeyframe(key)
            carpositionnode.parm('tx').set(longlist[x])
            carpositionnode.parm('ty').setKeyframe(key)
            carpositionnode.parm('ty').set(latlist[x])
        
        ##WHEEL ROTATION
        if x%5==0 or x==0 or x==totalframes-1:
            node = hou.node('/obj/model3_tire_front_left_tire_new/rotate_keyed')
            key = hou.Keyframe(x)
            node.parm('rx').setKeyframe(key)
            node.parm('rx').set(fullrotationatframe[x])

        ##G-FORCE PLOT 
        if x%5 == 0 or x == 0 or x == totalframes-1:##EVERY NTH FRAME SINCE IT'S OVERKILL TO KEYFRAME POSITION BY MILLISECOND
            node = hou.node('/obj/gforce/latlongaccel')
            key = hou.Keyframe(x)
            node.parm('value1v1').setKeyframe(key)
            node.parm('value1v1').set(longaccellist[x])
            
            key = hou.Keyframe(x)
            node.parm('value2v1').setKeyframe(key)
            node.parm('value2v1').set(lataccellist[x])
            ##MAX POWER!!
            node = hou.node('/obj/gforce/maxpower/maxpower_jpg')
            if abs(longaccellist[x]) > 9.81 or abs(lataccellist[x]) > 9.81:
                key = hou.Keyframe(x)
                node.parm('effectamount').setKeyframe(key)
                node.parm('effectamount').set(1)
            else:
                key = hou.Keyframe(x)
                node.parm('effectamount').setKeyframe(key)
                node.parm('effectamount').set(0)
                
        ##MIN AND MAX SPEED LABELS 
        if not minmaxspeedindexpoint > len(minmaxspeedlistpared)-1:##IF WE HAVEN'T ALREADY REACHED LAST INDEX POINT IN THE minmaxspeedlistpared LIST:
            if int(round(speedlist[x+1])) == minmaxspeedlistpared[minmaxspeedindexpoint]:##KEYFRAME THE HIDDEN VISIBILITY BEFORE WE TURN IT ON NEXT FRAME. NEEDS TO BE ROUNDED BECAUSE NOISY INTERPOLATED FRAMES BETWEEN KEYS COULD BE ABOVE OR BELOW VALUE
                #WE NEED TO CREATE THESE ON THIS FRAME SO THE VISIBILITY CAN BE KEYFRAMED
                fontnode = hou.node('obj/track').createNode('font', "font" + str(minmaxspeedindexpoint))
                fontnode.parm('text').set(str(minmaxspeedlistpared[minmaxspeedindexpoint]))          
                fontnode.parm('fontsize').set(0.002)
                fontnode.parm('file').set('Avenir Next Bold')
                fontnode.parm('tx').set(longlist[x])
                fontnode.parm('ty').set(latlist[x])
                fontnode.parm('tz').set(0.001)
                transformnode = hou.node('obj/track').createNode('xform', "transform" + str(minmaxspeedindexpoint))
                transformnode.parm('px').setExpression('ch("../font' + str(minmaxspeedindexpoint) + '/tx")')
                transformnode.parm('py').setExpression('ch("../font' + str(minmaxspeedindexpoint) + '/ty")')
                transformnode.parm('scale').set(0.151)
                transformnode.setNextInput(fontnode)#WIRE THEM TOGETHER
                visnode = hou.node('obj/track').createNode('visibility', 'visibility' + str(minmaxspeedindexpoint))
                visnode.setNextInput(transformnode)
                genode = hou.node('obj/track').createNode('groupexpression', 'groupexpression' + str(1))
                genode.parm('snippet1').set('@Frame>' + str(lapframes[laplist[x]]))
                ##DETERMINE LAP RANGE AND SET VISIBILITY ACCORDING TO WHAT LAP IT'S ON
                genode.setNextInput(visnode)
                visnodetoo = hou.node('obj/track').createNode('visibility', 'visibility' + str(minmaxspeedindexpoint) + 'too')
                visnodetoo.parm('group').set('group1')
                visnodetoo.setNextInput(genode)
                ##CONNECT THE VISNODE TO groupexpressoin THAT CONTROLS LAP-LEVEL VISIBILITY
                fontmergenode = hou.node('obj/track/fontmerge')
                fontmergenode.setNextInput(visnodetoo)#WIRE THEM TOGETHER
                
                ##KEYFRAMES
                key = hou.Keyframe(x)
                key.setExpression('constant()')
                visnode.parm('action').setKeyframe(key)
                visnode.parm('action').set(0)
                visnodetoo.parm('action').set(0)##ALWAYS HAVE THIS OFF 
                minmaxspeedindexpoint+=1##TICK UP SO WE CAN MOVE ON TO THE NEXT MATCH
                alreadysetbitflip = 0
            if int(round(speedlist[x])) == minmaxspeedlistpared[minmaxspeedindexpoint-1] and not alreadysetbitflip:
                visnode = hou.node('obj/track/visibility' + str(minmaxspeedindexpoint-1))
                ##FIND THE LAT AND LONG POSITION AND MOVE THE LABEL TO THE RIGHT OF THAT POINT
                key = hou.Keyframe(x)
                key.setExpression('constant()')
                visnode.parm('action').setKeyframe(key)
                visnode.parm('action').set(1)
                alreadysetbitflip = 1

                ##LAP DATA 
        lapdataholder = hou.node('/obj/track/lap_data')
        laptimeswitch = hou.node('/obj/lap_run_time/switch1')
        if x == 0 or laplist[x] > previouslap:
            key = hou.Keyframe(x)
            key.setExpression('constant()')
            lapdataholder.parm('value1v1').setKeyframe(key)
            lapdataholder.parm('value1v1').set(laplist[x])
            ##KEYFRAME THE font node SWITCH TO CHANGE THE INPUT DEPENDING ON THE LAP
            key = hou.Keyframe(x)
            laptimeswitch.parm('input').setKeyframe(key)
            laptimeswitch.parm('input').set(laplist[x])

        ##SET SPEEDOMETER KEY FRAMES
        speedholder = hou.node('/obj/speedometer/speed_holder')
        if speedlist[x] != previousspeed:##CREATE A KEYFRAME AT 0, BEFORE THE VALUE CHANGES AND ON VALUE CHANGE
            speedint = speedlist[x]
            key = hou.Keyframe(x)
            speedholder.parm('value1v1').setKeyframe(key)
            speedholder.parm('value1v1').set(speedint)

        previousspeed = speedlist[x]
