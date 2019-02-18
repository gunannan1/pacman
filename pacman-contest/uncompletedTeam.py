# myTeam1.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from captureAgents import CaptureAgent
import random, time, util
from game import Directions
import game
from util import nearestPoint


#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'BaseAgent', second = 'BaseAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########

class BaseAgent(CaptureAgent):

    def registerInitialState(self, gameState):
        self.start = gameState.getAgentPosition(self.index)
        CaptureAgent.registerInitialState(self, gameState)
        self.opponents=self.getOpponents(gameState)
        self.width=gameState.data.layout.width
        self.height=gameState.data.layout.height
        self.goHome=False
        self.offend=True
        self.layoutWidth = gameState.data.layout.width
        self.layoutHeight = gameState.data.layout.height
        self.team=self.getTeam(gameState)
        self.searchDepth=3
        for i in self.team:
            if i!=self.index:
                self.teammember=i
        x1,y1=self.start
        x2,y2=self.getPosition(self.teammember,gameState)
        if y2>y1:
            self.top=False
        else:
            self.top=True

        self.iegalPoints={}
        for i in range(self.layoutWidth):
            for j in range(self.layoutHeight):
                if not gameState.hasWall(i,j):
                    self.iegalPoints[(i,j)]=1




    def chooseAction(self, gameState):
        # print(self.foodListForMy(gameState))

        foodLeft = self.foodLeft(gameState)
        myFood=self.myFood(gameState)

        if(myFood<=4 and self.isPacman(self.index,gameState)):
            if self.minDistanceToOpponentPacman(gameState)[1]<8:
                self.offend=False
            else:
                return self.aStar(self.start,gameState)
        else:
            if(self.isGhost(self.index,gameState) and self.minDistanceToOpponentPacman(gameState)[1]<8
                    and not self.isScared(self.index,gameState)):
                self.offend=False
            else:
                self.offend=True

        if not self.isSafe(gameState) or foodLeft <= 2:
            return self.aStar(self.start,gameState)

        bestAction=self.bestAction(gameState)

        x, y = self.getPosition(self.index, gameState)
        dx, dy = game.Actions.directionToVector(bestAction)
        nextx, nexty = int(x + dx), int(y + dy)
        if(self.index==0):
            self.debugDraw((nextx, nexty), [1, 1, 0], clear=True)
        else:
            self.debugDraw((nextx, nexty), [1, 0, 1], clear=True)

        return bestAction

    def bestAction(self, gameState):
        actions = gameState.getLegalActions(self.index)
        actions.remove("Stop")
        values = [self.evaluate(gameState, a,self.offend) for a in actions]
        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]
        return random.choice(bestActions)

    def getSuccessor(self, gameState, action,index):
        successor = gameState.generateSuccessor(index, action)
        pos = successor.getAgentState(index).getPosition()
        if pos != nearestPoint(pos):
            return successor.generateSuccessor(index, action)
        else:
            return successor

    def evaluate(self, gameState, action,offend):
        """
        Computes a linear combination of features and feature weights
        """
        if offend is True:
            return self.alphaBetaSearch(gameState,action, 0, True, float("-inf"), float("inf"),self.index)
        else:
            features = self.getDefendFeatures(gameState, action)
            weights = self.getDefendWeights()
        return features * weights


    def isPacman(self,index,gameState):
        return gameState.getAgentState(index).isPacman

    def isGhost(self,index,gameState):
        return not gameState.getAgentState(index).isPacman

    def isScared(self, index, gameState):
        return gameState.getAgentState(index).scaredTimer>0

    def getScaredTime(self,index,gameState):
        return gameState.getAgentState(index).scaredTimer

    def getPosition(self,index,gameState):
        return gameState.getAgentState(index).getPosition()


    def minDistanceToOpponentGhost(self,gameState):
        min=float("inf")
        index=None
        for opponent in self.opponents:
            if self.getPosition(opponent,gameState) is not None and not self.isPacman(opponent,gameState):
                distance=self.getMazeDistance(self.getPosition(self.index,gameState),self.getPosition(opponent,gameState))
                if distance < min:
                    min=distance
                    index=opponent
        return [index,min]

    def minDistanceToOpponentPacman(self,gameState):
        min=float("inf")
        index=None
        for opponent in self.opponents:
            if self.getPosition(opponent,gameState) is not None and self.isPacman(opponent,gameState):
                distance=self.getMazeDistance(self.getPosition(self.index,gameState),self.getPosition(opponent,gameState))
                if distance < min:
                    min=distance
                    index=opponent
        return [index, min]

    def myFood(self,gameState):
        return len(self.getFoodYouAreDefending(gameState).asList())

    def foodLeft(self,gameState):
        return len(self.getFood(gameState).asList())

    def foodListForMy(self,gameState):
        foodlist=self.getFood(gameState).asList()
        # capsules = self.getCapsules(gameState)
        # if len(capsules)>0:
        #     for capsule in capsules:
        #         foodlist.append(capsule)
        sortList=[]
        for food in foodlist:
            sortList.append(food[1])
        list.sort(sortList)
        if len(sortList)>1:
            median=sortList[len(sortList)/2-1]
        else:
            median=0
        myFoolList=[]
        if self.top==True:
            for food in foodlist:
                if food[1]>median:
                    myFoolList.append(food)
        else:
            for food in foodlist:
                if food[1]<=median:
                    myFoolList.append(food)
        # print(capsules)
        # print(str(myFoolList)+","+str(self.index))
        if len(myFoolList)==0:
            myFoolList=foodlist
        return myFoolList

    def reachGoal(self,gameState):
        return self.foodLeft(gameState)<=2

    def needGoHome(self,gameState):
        return (self.reachGoal(gameState) or not self.pacmanIsSafe(gameState))and self.isPacman(self.index,gameState)

    def isSafe(self,gameState):
        return (self.isPacman(self.index,gameState) and self.pacmanIsSafe(gameState)) \
               or(not self.isPacman(self.index,gameState) and self.ghostIsSafe(gameState))

    def ghostIsSafe(self,gameState):
        # return (not self.IsScared(self.index,gameState) and self.minDistanceToOpponentGhost(gameState)[1]>2)
        return  self.minDistanceToOpponentGhost(gameState)[1] > 3


    def pacmanIsSafe(self,gameState):
        cloestGhost=self.minDistanceToOpponentGhost(gameState)
        return cloestGhost[1]>4 or \
               (self.isScared(cloestGhost[0], gameState) and self.getScaredTime(cloestGhost[0], gameState) > 2)



    def badAction(self, gameState, action):
        badPosition=[]
        opponents=self.opponents
        for i in opponents:
            pos=self.getPosition(i,gameState)
            if pos is not None and \
                    ((self.isPacman(self.index,gameState)and self.isGhost(i,gameState) and not self.isScared(i, gameState))
                     or (self.isGhost(self.index,gameState) and self.isScared(self.index, gameState))):
                badPosition.append(pos)
        if(len(badPosition)==0):
            return False
        else:
            x,y=self.getPosition(self.index,gameState)
            dx, dy = game.Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            for pos in badPosition:
                if (nextx, nexty)==pos:
                    return True
            return False

    def aStar(self,position,gameState):
        myPosition=self.getPosition(self.index,gameState)
        if(myPosition==position):
            return "Stop"
        h_cost=self.getMazeDistance(myPosition, position)
        g_cost=0
        cost = g_cost+h_cost
        actions = []
        startState = [myPosition, actions, gameState]
        priorqueue =util.PriorityQueue()
        priorqueue.push(startState, cost)
        explored = []

        enemyPos=None
        enemy=self.minDistanceToOpponentGhost(gameState)
        if enemy[0] !=None:
            enemyPos=self.getPosition(enemy[0],gameState)

        while True:
            if priorqueue.isEmpty():
                return "Stop"
            current = priorqueue.pop()
            currentPosition = current[0]
            currentaction = current[1]
            currentState=current[2]
            if currentPosition==position:
                # print(currentaction)
                # print(currentPosition)
                return currentaction[0]
            if currentPosition not in explored:
                explored.append(currentPosition)
                actions=currentState.getLegalActions(self.index)
                actions.remove("Stop")
                for action in actions:
                    nextAction=currentaction+[action]
                    nextSuccessor=self.getSuccessor(currentState,action,self.index)
                    myPosition=self.getPosition(self.index,nextSuccessor)
                    g_cost=len(nextAction)
                    # if((enemyPos!=None and myPosition==enemyPos) or self.killMySelf(currentState,action)):
                    if(self.badAction(currentState, action)):
                        h_cost=99999
                    else:
                        h_cost=self.getMazeDistance(myPosition, position)
                    cost=g_cost + h_cost
                    if myPosition not in explored:
                        priorqueue.push([myPosition,nextAction,nextSuccessor],cost)

    def estimateEnemyPosition(self,index,gameState):
        noiseDistance=gameState.getAgentDistances()[index]
        myPosition=self.getPosition(self.index,gameState)
        for point in self.iegalPoints.keys():
            if self.iegalPoints[point]!=0:
                distance=util.manhattanDistance(myPosition,point)
                probability=self.iegalPoints[point]*gameState.getDistanceProb(distance,noiseDistance)
                self.iegalPoints[point]=probability

        maxprobability=max(self.iegalPoints.values())
        maxPoints = [point for point in self.iegalPoints.keys() if self.iegalPoints[point] == maxprobability]
        # print(maxPoints)
        if maxprobability==0:
            for key in self.iegalPoints.keys():
                self.iegalPoints[key]=1
        point=random.choice(maxPoints)
        # print(point)
        return point

    def alphaBetaSearch(self, gameState, action, depth, isMaxPlayer, alpha, beta, index):
        successor = self.getSuccessor(gameState, action, index)
        if depth == self.searchDepth:
            return self.getOffendFeatures(successor)*self.getOffendWeights()

        if isMaxPlayer:
            bestValue = float("-inf")

            actions = successor.getLegalActions(self.index)
            for action in actions:
                value = self.alphaBetaSearch(successor, action, depth + 1, False, alpha, beta, self.index)
                bestValue = max(bestValue, value)
                alpha = max(alpha, bestValue)
                if beta <= alpha:
                    break
            return bestValue

        else:
            bestValue = float("inf")

            oppoents = []
            for e in self.opponents:
                if successor.getAgentPosition(e) != None:
                    oppoents.append(e)

            if len(oppoents) == 2:
                for action in successor.getLegalActions(oppoents[0]):
                    bestValue = min(bestValue, self.alphaBetaSearch(successor, action, depth + 1, True, alpha, beta,
                                                                    oppoents[0]))
                    beta = min(alpha, bestValue)
                    if beta <= alpha:
                        return bestValue
                for action in successor.getLegalActions(oppoents[1]):
                    bestValue = min(bestValue, self.alphaBetaSearch(successor, action, depth + 1, True, alpha, beta,
                                                                    oppoents[1]))
                    beta = min(alpha, bestValue)
                    if beta <= alpha:
                        return bestValue

            elif len(oppoents) == 1:
                for action in successor.getLegalActions(oppoents[0]):
                    bestValue = min(bestValue, self.alphaBetaSearch(successor, action, depth + 1, True, alpha, beta,
                                                                    oppoents[0]))
                    beta = min(alpha, bestValue)
                    if beta <= alpha:
                        return bestValue

            else:
                bestValue = min(bestValue, self.alphaBetaSearch(successor, "Stop", depth + 1, True, alpha, beta, index))
                beta = min(alpha, bestValue)
                if beta <= alpha:
                    return bestValue

            return bestValue

    def getOffendFeatures(self, gameState):
        features = util.Counter()
        foodList=self.foodListForMy(gameState)
        totalFoodlist=self.getFood(gameState).asList()
        capsules = self.getCapsules(gameState)
        if len(capsules) > 0:
            for capsule in capsules:
                totalFoodlist.append(capsule)
        myPos = gameState.getAgentState(self.index).getPosition()
        features['successorScore'] = len(totalFoodlist)  # self.getScore(successor)
        # Compute distance to the nearest food

        if len(foodList) > 0:  # This should always be True,  but better safe than sorry
            minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
            features['distanceToFood'] = minDistance
            # print(str(myPos) + "," + str(features) + "," + str(self.index) + "," + str(self.foodListForMy(gameState)))
        ghosts=self.minDistanceToOpponentGhost(gameState)
        if(ghosts[1]<5 and not self.isScared(ghosts[0],gameState)):
            features['distanceToGhost'] = ghosts[1]
        else:
            features['distanceToGhost'] = 1000000

        myfriend = self.teammember
        friendPos = gameState.getAgentState(myfriend).getPosition()
        if self.getMazeDistance(myPos, friendPos) <= 3:
            features['distanceToTeammate'] = self.getMazeDistance(myPos, friendPos)
        return features



    def getOffendWeights(self):
        return {'successorScore': -100, 'distanceToFood': -1,'distanceToGhost':100, 'distanceToTeammate': 3}

    def getDefendFeatures(self, gameState, action):

        features = util.Counter()
        successor = self.getSuccessor(gameState, action,self.index)

        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()

        # Computes whether we're on defense (1) or offense (0)
        features['onDefense'] = 1
        if myState.isPacman: features['onDefense'] = 0

        # Computes distance to invaders we can see
        enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
        # print(len(invaders))
        features['numInvaders'] = len(invaders)
        if len(invaders) > 0:
            dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
            # print(dists)
            features['invaderDistance'] = min(dists)
        else:
            dists=[]
            for i  in self.getOpponents(successor):
                enemyPosition=self.estimateEnemyPosition(i,successor)
                print(enemyPosition)
                dist=self.getMazeDistance(myPos, enemyPosition)
                dists.append(dist)
            features['invaderDistance'] = min(dists)
            print(features)

        if action == Directions.STOP: features['stop'] = 1
        rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
        if action == rev: features['reverse'] = 1

        return features

    def getDefendWeights(self):
        return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10, 'stop': -100, 'reverse': -2}


