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

#Gin and Tonic Agressive Version

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
        self.goHome=False
        self.offend=True
        self.team=self.getTeam(gameState)
        self.searchDepth=3

        #  the pacman whose y coordinate is larger at start state is top pacman
        for i in self.team:
            if i!=self.index:
                self.teammember=i
        x1,y1=self.start
        x2,y2=self.getPosition(self.teammember,gameState)
        if y2>y1:
            self.top=False
        else:
            self.top=True



    def chooseAction(self, gameState):

        myPosition=self.getPosition(self.index,gameState)
        foodLeft = self.foodLeft(gameState)
        foodLeftList=self.getFood(gameState).asList()
        ourFood=self.myFood(gameState)
        ourFoodList=self.getFoodYouAreDefending(gameState).asList()
        capsules = self.getCapsules(gameState)
        capsule = self.minDistanceToFood(gameState, capsules)[0]

        if self.reachGoal(gameState):
            if self.isGhost(self.index,gameState):
                self.offend=False
            else:
                return self.aStar(self.start, gameState,True)
        if not self.pacmanIsSafe(gameState)and self.isPacman(self.index,gameState):
            if len(capsules) != 0 :
                return self.aStar(capsule, gameState,False)
            else:
                if gameState.getAgentState(self.index).numCarrying < 2 and foodLeft!=0:
                    min=self.minDistanceToFood(gameState,foodLeftList)[0]
                    return self.aStar(min,gameState,False)
                return self.aStar(self.start, gameState,True)

        if ourFood<=3 and foodLeft>=6:
            if self.isPacman(self.index,gameState):
                if len(ourFoodList)!=0:
                    pos = self.minDistanceToFood(gameState, ourFoodList)[0]
                    return self.aStar(pos,gameState,True)
                else:
                    return self.aStar(self.start, gameState,True)
            elif self.minDistanceToOpponentPacman(gameState)[0] is not None:
                self.offend=False
            else:
                if len(ourFoodList) != 0:
                    pos = self.minDistanceToFood(gameState, ourFoodList)[0]
                    return self.aStar(pos, gameState,True)
                else:
                    return self.aStar(self.start, gameState, True)
        elif self.isGhost(self.index,gameState):
            if(self.isScared(self.index,gameState)):
                self.offend=True
            elif self.minDistanceToOpponentPacman(gameState)[1]<6:
                # print(str(self.minDistanceToOpponentPacman(gameState))+"  "+str(self.index))
                # print(self.getPosition(self.minDistanceToOpponentPacman(gameState)[0],gameState))
                self.offend = False
            elif not self.ghostIsSafe(gameState) or self.reachGoal(gameState) :
                self.offend = False
            else:
                self.offend = True
        else:
            self.offend=True


        # print(str(self.offend)+"  "+str(self.index))

        bestAction=self.bestAction(gameState)


        # show next position
        # x, y = self.getPosition(self.index, gameState)
        # dx, dy = game.Actions.directionToVector(bestAction)
        # nextx, nexty = int(x + dx), int(y + dy)
        # if(self.index==0):
        #     self.debugDraw((nextx, nexty), [1, 1, 0], clear=True)
        # else:
        #     self.debugDraw((nextx, nexty), [1, 0, 1], clear=True)

        return bestAction

    #  Choose best actions from actions
    def bestAction(self, gameState):
        actions = gameState.getLegalActions(self.index)
        for action in actions:
            if self.badAction(gameState,action):
                actions.remove(action)
        # actions.remove("Stop")
        values = [self.evaluate(gameState, a,self.offend) for a in actions]
        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]
        return random.choice(bestActions)

    # generate the successor of an agent after implementing an action
    def getSuccessor(self, gameState, action,index):
        successor = gameState.generateSuccessor(index, action)
        pos = successor.getAgentState(index).getPosition()
        if pos != nearestPoint(pos):
            return successor.generateSuccessor(index, action)
        else:
            return successor

    # use alphaBetaSearch to evaluate an action
    def evaluate(self, gameState, action,offend):
        """
        Computes a linear combination of features and feature weights
        """
        return self.alphaBetaSearch(gameState, action, 0, True, float("-inf"), float("inf"), self.index)

    # judge if is a pacman
    def isPacman(self,index,gameState):
        return gameState.getAgentState(index).isPacman

    # judge if is a ghost
    def isGhost(self,index,gameState):
        return not gameState.getAgentState(index).isPacman

    # judge if scared
    def isScared(self, index, gameState):
        return gameState.getAgentState(index).scaredTimer>0

    #get teh remain scare time
    def getScaredTime(self,index,gameState):
        return gameState.getAgentState(index).scaredTimer

    # get position
    def getPosition(self,index,gameState):
        return gameState.getAgentState(index).getPosition()

    # get min distance from myself to the ghost opponent
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

    # get min distance from myself to the pacman opponent
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

    # get min distance from myself to food
    def minDistanceToFood(self,gameState,foodList):
        min = float("inf")
        pos=None
        for food in foodList:
            distance=self.getMazeDistance(self.getPosition(self.index,gameState),food)
            if distance<min:
                min=distance
                pos=food
        return [pos,min]

    #get max distance from myself to food
    def maxDistanceToFood(self,gameState,foodList):
        max = -1
        pos=None
        for food in foodList:
            distance=self.getMazeDistance(self.getPosition(self.index,gameState),food)
            if distance>max:
                max=distance
                pos=food
        return [pos,max]

    # get left number of the food i am defending
    def myFood(self,gameState):
        return len(self.getFoodYouAreDefending(gameState).asList())

    #get left number of the food i want to eat
    def foodLeft(self,gameState):
        return len(self.getFood(gameState).asList())

    # use median of the y coordinate of foods to sperate to food
    # top pacman eat the foods higher than median, another pacman eat the foods lower than median
    # if one pacman's foodlist is empty, he can also eat another pacman's foods
    def foodListForMy(self,gameState):
        foodlist=self.getFood(gameState).asList()
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

    # reach the win conditon if food left less than two
    def reachGoal(self,gameState):
        return self.foodLeft(gameState)<=2

    # if foodleft less than two or he is chased by ghost, pacman need go home
    def needGoHome(self,gameState):
        return (self.reachGoal(gameState) or not self.pacmanIsSafe(gameState))and self.isPacman(self.index,gameState)

    # judge if a pacman or ghost is safe
    def isSafe(self,gameState):
        return (self.isPacman(self.index,gameState) and self.pacmanIsSafe(gameState)) \
               or(not self.isPacman(self.index,gameState) and self.ghostIsSafe(gameState))

    # is ghost is not scared and opponent's ghost is 3 grids distance to him, the ghost is safe
    def ghostIsSafe(self,gameState):
        enemyDis=self.minDistanceToOpponentGhost(gameState)[1]
        enemyIndex=self.minDistanceToOpponentGhost(gameState)[0]
        # return (not self.IsScared(self.index,gameState) and self.minDistanceToOpponentGhost(gameState)[1]>2)
        return  enemyDis > 3 or self.isScared(enemyIndex,gameState)

    # if the cloestghost is 5 or more distance to pacman or the ghost is scared, pacman is safe
    def pacmanIsSafe(self, gameState):
        cloestGhost = self.minDistanceToOpponentGhost(gameState)
        return cloestGhost[1] > 5 or \
               (self.isScared(cloestGhost[0], gameState) and self.getScaredTime(cloestGhost[0], gameState) > 5)

    # after implementing an action, if pacman kill itself, or the mazedistance to opponents is 1, this action is bad
    def badAction(self, gameState, action):
        badPosition=[]
        opponents=self.opponents
        for i in opponents:
            pos=self.getPosition(i,gameState)
            if pos is not None and \
                    ((self.isPacman(self.index,gameState)and self.isGhost(i,gameState) and not self.isScared(i, gameState))
                     or (self.isGhost(self.index,gameState) and self.isScared(self.index, gameState)) or
                     (self.isGhost(self.index,gameState) and self.isGhost(i,gameState))):
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
                if self.getMazeDistance((nextx,nexty),pos)==1:
                    return True
            return False

    # a star to find the path to a ceratain grid.
    def aStar(self,position,gameState,toHome):
        myPosition=self.getPosition(self.index,gameState)
        if(myPosition==position):
            return "Stop"
        h_cost=self.getMazeDistance(myPosition, position)
        g_cost=0
        cost = g_cost+h_cost
        actions = []
        startState = [myPosition, actions, gameState,cost]
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
            currentCost=current[3]

            # if pacman is not going home and all actions are bad action, the pacman will choose to go home
            if toHome == False and currentCost >= 99999:
                # print(self.index)
                return self.aStar(self.start, gameState, True)
            if currentPosition==position:
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
                    # after implementing the action,if pacman will kill itself ,the action will get the h cost 99999
                    # after implementing the action,if mazedistance from pacman to opponent ghost is 1, the action will get the h cost 10000
                    if(self.badAction(currentState, action)):
                        if enemyPos!=None and self.getMazeDistance(enemyPos,myPosition)==1:
                            h_cost=10000
                        else:
                            h_cost=99999
                    elif enemyPos!=None and self.getMazeDistance(enemyPos,myPosition)==1:
                        h_cost = 10000
                    else:
                        h_cost=self.getMazeDistance(myPosition, position)
                    cost=g_cost + h_cost
                    if myPosition not in explored:
                        priorqueue.push([myPosition,nextAction,nextSuccessor,cost],cost)

    # use alphaBetaSearch to evaluate ponits for a certain action in one state
    # We treat our agent as the maxising player and the oppoent(if we can observe) the minising player
    def alphaBetaSearch(self, gameState, action, depth, isMaxPlayer, alpha, beta, index):
        successor = self.getSuccessor(gameState, action, index)
        if depth == self.searchDepth:
            if self.offend==True:
                return self.getOffendFeatures(successor)*self.getOffendWeights()
            else:
                return self.getDefendFeatures(successor)*self.getDefendWeights()

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
                    
            # If the number of oppoents we can observe is 2, then pruning the tree based on each of the oppoent
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
                        
            # If the number of oppoents we can observe is 1, then pruning the tree based on that opponent

            elif len(oppoents) == 1:
                for action in successor.getLegalActions(oppoents[0]):
                    bestValue = min(bestValue, self.alphaBetaSearch(successor, action, depth + 1, True, alpha, beta,
                                                                    oppoents[0]))
                    beta = min(alpha, bestValue)
                    if beta <= alpha:
                        return bestValue
                        
            # If no oppoents observed, then pruning the tree based on the successor self doing Stop action
            else:
                bestValue = min(bestValue, self.alphaBetaSearch(successor, "Stop", depth + 1, True, alpha, beta, index))
                beta = min(alpha, bestValue)
                if beta <= alpha:
                    return bestValue

            return bestValue

    # offendfeatures, similar to baseline.But we give the feature according to the distance from our pacman to opponents' ghost
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

        return features


    # offendWeights, similar to baseline
    def getOffendWeights(self):
        return {'successorScore': -100, 'distanceToFood': -1,'distanceToGhost':100}

    # defendfeatures, similar to baseline
    def getDefendFeatures(self, gameState):

        features = util.Counter()
        # successor = self.getSuccessor(gameState, action,self.index)

        myState = gameState.getAgentState(self.index)
        myPos = myState.getPosition()

        # Computes whether we're on defense (1) or offense (0)
        features['onDefense'] = 1
        if myState.isPacman: features['onDefense'] = 0

        # Computes distance to invaders we can see
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
        # print(len(invaders))
        features['numInvaders'] = len(invaders)
        if len(invaders) > 0:
            dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
            # print(dists)
            features['invaderDistance'] = min(dists)
        else:
            features['invaderDistance'] = 100


        return features

    # defendweights, similar to baseline
    def getDefendWeights(self):
        return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10}


