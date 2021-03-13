from tkinter import *
from cmu_112_graphics import *
import math
from bisect import bisect_left

# Has various classes used to create graphs using tkinter canvas


class StackedTimeGraph():

    def __init__(self, timeData, yDataLists, xDims, yDims):
        self.graphs = {}
        self.border = 40
        yDecrement = abs(yDims[0] - yDims[1])/len(yDataLists)
        graphYOrigin = yDims[0]
        for yData in yDataLists:
            graphYDims = (graphYOrigin, graphYOrigin - yDecrement)
            graphYOrigin -= yDecrement
            self.graphs[yData.label] = Graph(
                timeData, yData, xDims, graphYDims, border=self.border)

    def draw(self, canvas):
        for graphKey in self.graphs:
            graph = self.graphs[graphKey]
            graph.draw(canvas)

    def updateHover(self, x):
        for graphKey in self.graphs:
            self.graphs[graphKey].selectPoint(x)


class Graph():

    def __init__(self, xData, yData, xDims, yDims, title="", border=None):
        self.cXOrigin, self.cYOrigin = xDims[0], yDims[0]
        self.width = abs(xDims[1] - xDims[0])
        self.height = abs(yDims[0] - yDims[1])
        self.title = title
        self.xAxis = Axis(xData)
        self.yAxis = Axis(yData)
        self.backgroundColor = "white"
        if border is not None:
            self.border = border
        else:
            self.border = 0.1*self.height
        self.selectedIndex = None

    def draw(self, canvas):
        self.xAxis.checkScale()
        self.yAxis.checkScale()
        self.drawAxes(canvas)

        dataPoints = min(len(self.xAxis.data.data), len(self.yAxis.data.data))
        for i in range(dataPoints-1):
            x1, y1 = self.xAxis.data.data[i], self.yAxis.data.data[i]
            x2, y2 = self.xAxis.data.data[i+1], self.yAxis.data.data[i+1]
            if x1 is None or y1 is None or x2 is None or y2 is None:
                continue
            self.createLine(canvas, x1, y1, x2, y2, fill="red")

        if self.selectedIndex is not None:
            self.drawSelectedPoint(canvas, self.selectedIndex)

    def drawAxes(self, canvas):
        canvas.create_rectangle(self.cXOrigin, self.cYOrigin,
                                self.cXOrigin+self.width, self.cYOrigin-self.height,
                                fill=self.backgroundColor)
        self.createLine(canvas, self.xAxis.scaleMin, self.yAxis.scaleMin,
                        self.xAxis.scaleMin+self.xAxis.scaleRange, self.yAxis.scaleMin,
                        fill="blue")
        for xScale in self.xAxis.scale:
            x, y = self.graphToCanvasCoords(xScale, self.yAxis.scaleMin)
            canvas.create_text(x, y, text=str(xScale), anchor="n")
        self.createLine(canvas, self.xAxis.scaleMin, self.yAxis.scaleMin,
                        self.xAxis.scaleMin, self.yAxis.scaleMin+self.yAxis.scaleRange,
                        fill="blue")
        for yScale in self.yAxis.scale:
            x, y = self.graphToCanvasCoords(self.xAxis.scaleMin, yScale)
            canvas.create_text(x, y, text=str(yScale), anchor="e")
        canvas.create_text(self.cXOrigin+self.width/2, self.cYOrigin-self.height,
                           anchor="n", text=self.yAxis.label + " vs " + self.xAxis.label)

    def drawSelectedPoint(self, canvas, index):
        r = 5
        if index >= self.yAxis.data.len:
            return
        yVal = self.yAxis.data.data[index]
        cX, cY = self.graphToCanvasCoords(self.xAxis.data.data[index], yVal)
        canvas.create_oval(cX+r, cY+r, cX-r, cY-r, fill="red")
        canvas.create_text(cX, cY, anchor="sw", text=str(round(yVal, 3)))

    def graphToCanvasCoords(self, x, y):
        xScaled = (x-self.xAxis.scaleMin) / self.xAxis.scaleRange
        cX = xScaled * (self.width-(2*self.border)) + \
            self.cXOrigin + self.border
        yScaled = -(y-self.yAxis.scaleMin) / self.yAxis.scaleRange
        cY = yScaled * (self.height-(2*self.border)) + \
            self.cYOrigin - self.border
        return cX, cY

    def createLine(self, canvas, x1, y1, x2, y2, **kwargs):
        cX1, cY1 = self.graphToCanvasCoords(x1, y1)
        cX2, cY2 = self.graphToCanvasCoords(x2, y2)
        canvas.create_line(cX1, cY1, cX2, cY2, **kwargs)

    def appToGraphCoords(self, x):
        xOriginAdjusted = x-self.cXOrigin-self.border
        xGraph = (xOriginAdjusted/(self.width-self.border*2) *
                  self.xAxis.scaleRange)+self.xAxis.scaleMin
        return xGraph

    def selectPoint(self, appX):
        time = self.appToGraphCoords(appX)
        index = bisect_left(self.xAxis.data.data, time)
        if time > self.xAxis.scaleMin:
            self.selectedIndex = index
        else:
            self.selectedIndex = None


class Axis():

    def __init__(self, data, increments=2):
        self.data = data
        self.label = self.data.label
        self.increments = increments
        self.setScale()

    def setScale(self):
        minRange = self.data.max - self.data.min
        if minRange == 0:
            minRange = 1
        average = self.roundSigFigs(
            self.data.max+self.data.min, 2, math.ceil)/2
        increment = self.roundSigFigs(minRange, 2, math.ceil)/self.increments
        middle = self.roundSigFigs(average, 2, round)
        rangeHalf = int(self.increments/2)
        self.scale = []
        for i in range(-rangeHalf, rangeHalf+1):
            rounded = self.roundSigFigs(middle + (increment*i), 2, round)
            self.scale.append(round(rounded, 2))
        self.scaleMin = self.scale[0]
        self.scaleRange = self.scale[-1] - self.scaleMin

    def roundSigFigs(self, x, sigs, roundFunc):
        if x == 0:
            return 0
        power = math.floor(math.log10(abs(x)))
        floatifiedX = x/10**(power-sigs+1)
        rounded = roundFunc(floatifiedX) * 10**(power-sigs+1)
        return math.copysign(rounded, x)

    def checkScale(self):
        if self.scale[-1] < self.data.max \
                or self.scale[0] > self.data.min:
            self.setScale()
        elif self.data.max-self.data.min != 0.0 and \
                self.scale[-1] - self.data.max > (self.data.max-self.data.min)*0.2:
            self.setScale()


if __name__ == "__main__":
    data = [-2.0, 0.1, 0.2, 360]
    axis = Axis(data, 10)
