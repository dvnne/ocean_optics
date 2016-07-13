# Plotting and calculation of vegetation indices for reflectance data
# Added: graphical file pickers
# Update: plotFile allows for multiple selections
# Updtae: Tkinter window allowing for data summary and chart creation

from __future__ import division
import string, os, csv
from shutil import rmtree
import Tkinter as tk
from Tkinter import Tk
from tkFileDialog import *
from matplotlib.pyplot import *

def getMetrics(path):
    """Takes a file and returns a dictionary of {metric: value,...}"""
    d = dict()
    lines = readFile(path)
    lines = splitfile(lines)
    makeData(lines) # okay, now we have global data
    metrics = [NDVI, PRI, MCARI, TCARI, OSAWI, NPCI, gitl550, gitl700]
    for metric in metrics:
        d[metric.__name__] = metric()
    return d

def plotFile(leg = True):
    """
    Prompts for 'plots' number of files to open and plots them all on
    one chart
    """
    paths = askopenfilenames()
    if type(paths) != tuple: # for version issues
        paths = paths.split()
    if len(paths) < 1: return # stop here if canceled
    for path in paths:
        name = getFileName(path)
        lines = readFile(path)
        lines = splitfile(lines)
        x, y = makeData(lines, glob = False)
        plot(x, y, label = name)
    ylim(0, 100)
    xlim(400, 1000)
    xlabel("Wavelength (nm)")
    ylabel("Reflection (%)")
    if leg: legend()
    if len(paths) > 0: show()

def getFileName(path):
    """
    Strips a full path down to the base file name
    """
    for i in xrange(len(path) - 1, -1, -1):
        if path[i] == '\\' or path[i] == '/':
            return str(path[i + 1:])
    return str(path)

def summarizeData():
    """
    Calculate metrics for all files in a directory and makes
    2 files (csv and txt) with data sumamrized and returns a
    dictionary with summarized data
    """
    directory = askdirectory()
    finaldir = str(directory + os.sep + 'indices')
    try:
        rmtree(finaldir)
    except:
        pass
    finally:
        os.mkdir(finaldir)
    d = dict()
    foutbase = finaldir + os.sep + getFileName(directory) + 'Indices'
    for f in os.listdir(directory)[:]: # make a copy
    # create a dictionary of {filename: {metric: value,...}, ...}
        if f[-4:] != '.txt': # skip over incompatable filess
            continue
        filename = f
        f = directory + os.sep + f
        d[filename] = getMetrics(f)
    l = makeLines(d) # l must be comprised of strings
    writeLines(l, foutbase + '.txt')
    writeCSV(d, foutbase + '.csv')
    return d

def writeLines(l, path):
    """Write lines of list l to file given by 'path'"""
    assert type(l) == list
    newfile = path
    assert os.path.exists(path) == False
    with open(newfile, 'w+') as fout:
        fout.writelines(l)
    print "File written to %s" % newfile

def writeCSV(d, path):
    """
    Write dictionary of dictionaries to .csv file (excel dialect) given by path
    """
    fields = ['filename', 'NDVI', 'PRI', 'MCARI', 'TCARI', 'OSAWI', 'NPCI', 
                'gitl550', 'gitl700']
    l = flattendict(d)
    with open(path, 'w+b') as csvout:
        writer = csv.DictWriter(csvout, fieldnames = fields)
        writer.writeheader()
        for dct in sorted(l, key = lambda elem: elem['filename']):
            writer.writerow(dct)
    print "File written to %s" % path

def flattendict(d):
    """
    Takes a dictionary of dictionaries and returns a list of dictionaries
    with the original keys remapped to 'filename'
    """
    l = list()
    for key in d:
        wrapperdict = {'filename': key}
        for kkey in d[key]:
            wrapperdict[kkey] = d[key][kkey]
        l.append(wrapperdict)
    return sorted(l, key = lambda dct: dct['filename'])

def makeLines(d): # This doc-strng is wrong
    """
    Takes a dictionary of dictionaries and returns a list of
    [key, subkey\tsubvalue,..., '' (empty string),...]
    It's ugly but at the end of each append call a newline is added to
    correct for my stupidity
    """
    l = list()
    for key in sorted(d.keys()):
        assert type(d[key]) == dict
        l.append(key + '\n')
        for kkey in d[key]:
            l.append(kkey + '\t' + str(d[key][kkey]) + '\n')
        l.append('\n')
    if len(l) > 0: l.pop() # remove last new line
    return l

def splitfile(lines):
    """
    Takes a list of lines from a read file and returns
    the list with only the raw data as a list of strings
    """
    if hasHeaderData(lines) == True: # hardcoded for Ocean View software
        start = 14
    else:
        start = 0
    while isData(lines[-1]) == False:
        lines.pop()
    return lines[start:]

def isData(line):
    """Takes a line (string) from a read file and determines if it is data"""
    if line.isspace() or line == '': return False
    for c in line:
        if c.upper() in string.ascii_uppercase:
            return False
    return True

def hasHeaderData(lines):
    # Checks if the first line has letters - it's hardcoded but whatever
    if isData(lines[0]):
        return False
    else:
        return True

def readFile(target):
    """Read the lines of a file (properly) and return the lines"""
    assert os.path.exists(target)
    with open(target) as fin:
        lines = fin.read()
    lines = lines.splitlines()
    return lines

def makeData(datatxt, glob = True):
    """
    Creates a list of tuples for the data in the 
    lines of the read file
    """
    if glob: global data
    x, y = [], []
    data = []
    for line in datatxt:
        datapoint = [float(n) for n in line.split()]
        assert len(datapoint) == 2
        data.append(tuple(datapoint))
        x.append(datapoint[0])
        y.append(datapoint[1])
    data.sort(key = lambda l: l[0])
    return (x, y)

def nearestElement(l, target):
    # Right now only works for sorted list of tuples -- potential changes:
    # -No list of tuples requirement
    # -List can be unsorted
    # -Pick index for comparison
    """
    Takes a sorted list of tuples and returns the index of the 
    tuple with first element closest to 'target'
    """
    difflist = list()
    for t in l:
        val = t[0]
        diff = abs(val - target)
        difflist.append(diff)
    for i in xrange(len(difflist)):
        if (i < len(difflist) - 1 and difflist[i] < difflist[i + 1]):
            return i
    return len(difflist) - 1 # index of last element

################ Metrics #################
# Taking no parameters these calculate the metrics using
# the 'R' function to pick out the reflectance needed
#########################################

def R(wavelength):
    """Reflectance at 'wavelength'"""
    index = nearestElement(data, wavelength)
    return data[index][1]

def NDVI():
    """Normalized Difference Vegetation Index:
    (R780 - R670)/(R780 + R670)"""
    R670, R780 = R(670), R(780)
    return (R780-R670)/(R780 + R670)

def PRI():
    """Photochemical Reflectance Index 
    (R531 - R570)/(R531 + R570)"""
    R531, R570 = R(531), R(570)
    return (R531 - R570)/(R531 + R570)

def MCARI():
    """[(R700 - R670) - 0.2*(R700 - R550)]*(R700/R670)"""
    R700, R670, R550 = R(700), R(670), R(550)
    return ((R700 - R670) - 0.2*(R700 - R550))*(R700/R670)

def TCARI():
    """Transformed Chlorophyll Absorption in Reflectance Index:
    3*[(R700 - R650) - 0.2*(R700 - R550)(R700/R670)]"""
    R700, R670, R650, R550 = R(700), R(670), R(650), R(550)
    return 3*((R700 - R670) - 0.2*(R700 - R550)*(R700/R670))

def OSAWI():
    """Optimized Soil Adjustment Index:
    (1 + 0.16*(R800 - R670))/(0.16 + R760 + R800)"""
    R670, R760, R800 = R(670), R(760), R(800)
    return (1.16*(R800 - R670))/(0.16 + R760 + R800)

def NPCI():
    """Normalized Pigment Chlorophyll Index:
    (R680-R430)/(R680 + R430)"""
    R430, R680 = R(430), R(680)
    return (R680-R430)/(R680 + R430)

def gitl550():
    """R750/R550"""
    R750, R550 = R(750), R(550)
    return R750/R550

def gitl700():
    """R750/R700"""
    R700, R750 = R(700), R(750)
    return R750/R700
    
##############Test Functions###################

def testWriteLines(): # writeLines not meant to work with non-string objects
    print "Testing writeLines............."
    l1 = list()
    # l2 = [1]
    l3 = ['']
    # l4 = [1, 'str']
    l5 = ['s1\n', 's2\n']
    path = 'WriteLines'
    if os.path.exists(path):
        for f in os.listdir(path): os.remove(path + os.sep + f)
        os.rmdir(path)
    os.mkdir(path)
    pathBase = path + os.sep
    writeLines(l1, pathBase + 'l1.txt')
    # writeLines(l2, pathBase + 'l2.txt')
    writeLines(l3, pathBase + 'l3.txt')
    # writeLines(l4, pathBase + 'l4.txt')
    writeLines(l5, pathBase + 'l5.txt')
    print

def testmakeLines():
    print "Testing makeLines............."
    empty = {}
    assert makeLines(empty) == []
    d1 = {'key1': {'kkey1': 11}}
    assert makeLines(d1) == ['key1\n', 'kkey1\t11\n']
    d1['key1']['kkey2'] = 12
    print d1
    print '\t-->', makeLines(d1)
    d2 = {'key1': {'kkey1': 11, 'kkey2': 12}, 'key2': {'kkey21': 21}}
    print d2
    print '\t-->', makeLines(d2)
    print

def testReadFile():
    print "Testing readFile............",
    assert readFile('test.txt') == [str(i) for i in range(5)]
    assert readFile('empty.txt') == []
    print 'passed!\n'

def testNearestElement():
    print "Testing nearestElement..........",
    from random import random
    l = [(i, 'foo') for i in range(10)]
    assert nearestElement(l, 4) == 4
    assert nearestElement(l, 90) == 9
    assert nearestElement(l, 0) == 0
    assert nearestElement(l, 90) == 9
    l = [(i + random(), 'foo') for i in range(0, 91, 10)]
    assert nearestElement(l, 21) == 2
    assert nearestElement(l, 0) == 0
    assert nearestElement(l, -6) == 0
    assert nearestElement(l, -400) == 0
    assert nearestElement(l, 90) == 9
    assert nearestElement(l, 89) == 9
    assert nearestElement(l, 100) == 9
    l = [(i, 'foo') for i in range(-5, 6)]
    assert nearestElement(l, 4) == l.index((4, 'foo'))
    assert nearestElement(l, -3) == l.index((-3, 'foo'))
    print 'passed!\n'

def testFlattendict():
    print "Testing flattendict............" 
    d = dict()
    assert flattendict(d) == []
    d = {'empty': {}}
    assert flattendict(d) == [{'filename': 'empty'}]
    d = {'key1': {'kkey11': 11}}
    assert flattendict(d) == [{'filename': 'key1', 'kkey11': 11}]
    d = {'key1': {'kkey1': 11},'key2': {'kkey21': 21, 'kkey22': 22}}
    print flattendict(d)

def testGetFileName():
    print 'Testing getFileName.........',
    assert getFileName('') == ''
    assert getFileName('foo.py') == 'foo.py'
    assert getFileName(u'DaVonne/foo.py') == 'foo.py'
    assert getFileName(('Hogwarts\Students\Potter.jkrowling') ==
                        'Potter.jkrowling')
    print 'passed!'

def testAll():
    testWriteLines()
    testmakeLines()
    testReadFile()
    testNearestElement()

def run():
    root = Tk()
    initAnimation(root)
    root.mainloop()

def initAnimation(root):
    # Define Buttons
    legend = tk.IntVar()
    legendButton = tk.Checkbutton(root, text = "Show Legend", variable = legend)
    plotButton = tk.Button(root, text = "Plot",
                           command = lambda : plotFile(leg = legend.get()))
    summarizeButton = tk.Button(root, text = "Summarize Directory",
                                command = summarizeData)
    legendButton.pack()
    plotButton.pack()
    summarizeButton.pack()

if __name__ == '__main__':
    run()

