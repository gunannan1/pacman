# baselineTeam.py
# ---------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

from captureAgents import CaptureAgent
import distanceCalculator
import random, time, util
from game import Directions
from game import *
from util import nearestPoint
import copy


#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'SmartReflexAgent', second = 'SmartReflexAgent'):
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
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########

class ReflexCaptureAgent(CaptureAgent):
  """
  A base class for reflex agents that chooses score-maximizing actions

  """
  def getSuccessor(self, gameState, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()
    if pos != nearestPoint(pos):
      # Only half a grid position was covered
      return successor.generateSuccessor(self.index, action)
    else:

      return successor

  def distributeFood(self,gameState):
       foodlists=self.createFoodlists(gameState)
       while len(foodlists['Middle'])>0:
           bottomFoodNum=len(foodlists['Bottom'])
           topFoodNum = len(foodlists['Top'])
           if bottomFoodNum < topFoodNum:
               newFood, _ = self.getClosestFoodFromLists(foodlists['Middle'], foodlists['Bottom'])
               foodlists['Bottom'].append(newFood)
           else:
               newFood, _ = self.getClosestFoodFromLists(foodlists['Middle'], foodlists['Top'])
               foodlists['Top'].append(newFood)

           foodlists['Middle'].remove(newFood)
       return foodlists


  def createFoodlists(self, gameState):
      foodlists={'Bottom':[],'Top':[],'Middle':[]}
      self.mapWidth=gameState.data.layout.width
      self.mapHeight=gameState.data.layout.height

      foodPos=self.getFood(gameState).asList()
      for x, y in foodPos:
          topLimit=self.mapHeight-10
          if y<6:
              foodlists['Bottom'].append((x,y))
          elif y>topLimit:
              foodlists['Top'].append((x,y))
          else:
              foodlists['Middle'].append((x,y))
      return foodlists

  def getClosestFoodFromLists(self, sourcelist, destlist):
      closestFoodDist = 9999999
      closestFoodPos = None
      for food in destlist:
          currFoodPos, currFoodDist = self.getClosestFood(sourcelist, food)
          if closestFoodPos is None or currFoodDist < closestFoodDist:
              closestFoodPos = currFoodPos
              closestFoodDist = currFoodDist
      return (closestFoodPos, closestFoodDist)

  def getClosestFood(self, foodList, pos):
      closestFoodDist = 9999999
      closestFoodPos = None
      for food in foodList:
          currFoodDist = self.distancer.getDistance(food, pos)
          if closestFoodPos is None or currFoodDist < closestFoodDist:
              closestFoodPos = food
              closestFoodDist = currFoodDist
      return (closestFoodPos, closestFoodDist)

  def getClosestGhost(self, gameState, currentPos, threshold):
      ghostList=self.getOpponents(gameState)
      closeGhost=[]
      for ghost in ghostList:
          if gameState.getAgentPosition(ghost)!=None:
              ghostPos=gameState.getAgentState(ghost).getPosition()
              if self.getMazeDistance(currentPos,ghostPos) <=threshold:
                  closeGhost.append(ghost)
      return closeGhost

  def getCloestPacman(self, gameState, currentPos, threshold):
      pacmanList=self.getOpponents(gameState)
      closePacman=[]
      for pacman in pacmanList:
          if gameState.getAgentPosition(pacman)!=None:
              pacmanPos=gameState.getAgentState(pacman).getPosition()
              if self.getMazeDistance(currentPos, pacmanPos) <=threshold:

                  closePacman.append(pacman)
      return closePacman






class SmartReflexAgent(ReflexCaptureAgent):
  """
  A reflex agent that seeks food. This is an agent
  we give you to get an idea of what an offensive agent might look like,
  but it is by no means the best or only way to build an offensive agent.
  """
  def registerInitialState(self, gameState):
      CaptureAgent.registerInitialState(self,gameState)

      self.teamIndex=self.getTeam(gameState)
      self.enemyIndex=self.getOpponents(gameState)
      self.walls=gameState.getWalls().asList()
      self.isBottom=(self.index==min(self.teamIndex))
      self.foodlists=self.distributeFood(gameState)
      self.halflayout=gameState.data.layout.height/2
      self.estimateEnemyPos = {}


      #bottom pacman eat bottom food
      if self.isBottom:
          self.foodlist=self.foodlists['Bottom']
      else:
          self.foodlist=self.foodlists['Top']

      self.position=gameState.getAgentPosition(self.index)
      self.searchDepth=3
      self.isAttacking=True
      for enemy in self.getOpponents(gameState):
          self.estimateEnemyPos[enemy] = inferringGoalProbability(gameState, self.index, enemy)

  def chooseAction(self, gameState):
      #find the closest ghost
      for opponent in self.getOpponents(gameState):
          self.estimateEnemyPos[opponent].observe(gameState)

      closeGhost1=self.getClosestGhost(gameState, self.position,2)

      foodLeft = len(self.getFood(gameState).asList())

      ourfoodLeft = self.getFoodYouAreDefending(gameState).asList()

      if ((len(ourfoodLeft) >5 or self.getScore(gameState)==0) or (self.index<2 and len(self.foodlist)>3)) and not (not gameState.getAgentState(self.index).isPacman and len(closeGhost1)>0):
          self.isAttacking=True

      else:
          self.isAttacking=False

      if self.isAttacking:
          closeGhost=self.getClosestGhost(gameState,self.position,4)
          #use A* to return home if pacman has already eaten food and meet ghost
          if gameState.getAgentState(self.index).isPacman and (foodLeft<=2 or len(closeGhost) >0):
              return self.returnHome(self.position)

          #when in ghost mode, detect enemy pacman in our domain
          if not gameState.getAgentState(self.index).isPacman:
              closeGhost1=self.getClosestGhost(gameState,self.position,5)
              detectedPacman=len(self.getCloestPacman(gameState,self.position,5))
              if detectedPacman >0:
                  self.isAttacking=False

      actions = gameState.getLegalActions(self.index)
      values = [self.evaluate(gameState, action) for action in actions]
      maxValue = max(values)
      bestActions = [action for action, value in zip(actions, values) if value == maxValue]
      return random.choice(bestActions)


  def evaluate(self, gameState,action):
      if self.isAttacking:
          successor=self.getSuccessor(gameState,action)
          #successor=gameState.generateSuccessor(self.index,action)
          actions=gameState.getLegalActions(self.index)
          alpha= float("-inf")
          beta= float("inf")
          depth=0
          evaluatePoint= self.AlphaBetaPruning(successor, depth)


          return evaluatePoint

      else:
          features=self.getFeaturesAsDefender(gameState, action)
          weights=self.getWeightsAsDefender(gameState, action)
          evaluatePoint=features*weights
      return evaluatePoint

  def getSuccessors(self, curPosition, gameState):
      successors = []
      walls =copy.copy(self.walls)

      # When our agent is Pacman mode, treat chasing ghost like a wall:
      if gameState.getAgentState(self.index).isPacman:
          enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
          ghosts = [e for e in enemies if not e.isPacman and e.getPosition() != None and e.scaredTimer <= 0]
          if len(ghosts)>0:

              ghostpositions = [g.getPosition() for g in ghosts]
              walls.extend(ghostpositions)

      for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
          x, y = curPosition
          dx, dy = Actions.directionToVector(direction)
          nextx, nexty = int(x + dx), int(y + dy)
          if (nextx, nexty) not in walls:
              newState=(nextx,nexty)
              successors.append((newState, direction))
      return successors

  def returnHome(self, position):
      pqueue = util.PriorityQueue()
      gameState = self.getCurrentObservation()
      curPosition = gameState.getAgentPosition(self.index)
      visited = []
      pqueue.push((curPosition, []), self.getMazeDistance(curPosition, position))
      while not pqueue.isEmpty():
          curentPosition, actions = pqueue.pop()
          if curentPosition == position:
              return actions[0]
          if curentPosition not in visited:
              visited.append(curentPosition)
              successors = self.getSuccessors(curentPosition, gameState)
              for successor in successors:
                  newactions = actions + [successor[1]]
                  pqueue.push((successor[0], newactions), len(newactions) + self.getMazeDistance(
                      successor[0], position))
      return Directions.STOP

  def AlphaBetaPruning(self, successor, depth):
      alpha=-99999999999
      beta=99999999999
      return self.minValue(successor, alpha, beta, depth)

  def maxValue(self, gameState, alpha, beta, depth):
      if depth==self.searchDepth:
          return self.evaluateSuccessor(gameState)

      v=-9999999999999
      actions=gameState.getLegalActions(self.index)

      for action in actions:
          successor=self.getSuccessor(gameState, action)
          V=self.minValue(successor, alpha, beta, depth+1)

          if V>v:
              v=V
          if V>=beta:
              return v
          if V>alpha:
              alpha=V

      return v


  def minValue(self, gameState, alpha, beta, depth):
      if depth==self.searchDepth:
          return self.evaluateSuccessor(gameState)

      v=9999999999999
      actions=gameState.getLegalActions(self.index)

      for action in actions:
          successor=self.getSuccessor(gameState,action)
          V=self.maxValue(successor,alpha,beta,depth+1)
          if V<v:
              v=V
          if V<=alpha:
              return v
          if V<beta:
              beta=V
      return v

  def evaluateSuccessor(self, gameState):
      foodList=self.getFood(gameState).asList()
      capsules=self.getCapsules(gameState)

      if capsules is not None:
          for c in capsules:
              foodList.append(c)

      foodListBottom=[]
      foodListTop=[]
      for food in foodList:
          if food[1]<=self.halflayout:
              foodListBottom.append(food)
          else:
              foodListTop.append(food)

      gameScore=-len(foodList)
      minDistance=None
      ghostScore =None

      if len(foodList) > 0:
          myPos = gameState.getAgentState(self.index).getPosition()
          if self.index < 2:
              if len(foodListBottom) > 0:
                  minDistance = min([self.getMazeDistance(
                      myPos, food) for food in foodListBottom])
              else:
                  minDistance = min([self.getMazeDistance(
                      myPos, food) for food in foodList])
          else:
              if len(foodListTop) > 0:
                  minDistance = min([self.getMazeDistance(
                      myPos, food) for food in foodListTop])
              else:
                  minDistance = min([self.getMazeDistance(
                      myPos, food) for food in foodList])

      ghostScore = len(self.getClosestGhost(gameState,myPos,4))

      if ghostScore > 0:
          ghostScore = 1
      else:
          ghostScore = 0

      if minDistance is not None:
          return 100 * gameScore + (-1 * minDistance) + (-10000 * ghostScore)
      else:
          return 100 * gameScore + (-10000 * ghostScore)

#--------------defence mode------------------------

  def getFeaturesAsDefender(self, gameState, action):
      features = util.Counter()

      successor = self.getSuccessor(gameState, action)

      nextState = gameState.generateSuccessor(self.index, action)
      nextStatePos = nextState.getAgentPosition(self.index)

      myState = successor.getAgentState(self.index)
      myPos = myState.getPosition()

      features['onDefense'] = 1
      if myState.isPacman: features['onDefense'] = 0

      enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
      enemyPacman = [a for a in enemies if a.isPacman and a.getPosition() != None]
      features['numInvaders'] = len(enemyPacman)

      # "here use the inference module to get the estimate oppoent distance"

      likelyPos = {}
      for enemy in self.getOpponents(gameState):
          likelyPos[enemy] = self.estimateEnemyPos[enemy].belief.argMax()

      minDistanceToPacman = 9999999
      minDistanceToGhost = 9999999

      for enemy in self.getOpponents(gameState):
          distanceToEnemy = self.getMazeDistance(myPos, likelyPos[enemy])
          if successor.getAgentState(enemy).isPacman:
              minDistanceToPacman = min(distanceToEnemy, minDistanceToPacman)
          else:
              minDistanceToGhost = min(distanceToEnemy, minDistanceToGhost)

      if minDistanceToPacman != 9999999:
          features['minDistanceToPacman'] = minDistanceToPacman
      else:  # all ghost
          features['minDistanceToPacman'] = minDistanceToGhost

      if action == Directions.STOP: features['stop'] = 1
      rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
      if action == rev: features['reverse'] = 1

      return features

  def getWeightsAsDefender(self, gameState, action):
      return {'numInvaders': -1000, 'onDefense': 100, 'minDistanceToPacman': -10, 'stop': -100, 'reverse': -2}

class inferringGoalProbability:
    def __init__(self,gameState,myIndex,enemyIndex):
        self.index = myIndex
        self.enemyIndex = enemyIndex
        self.belief = util.Counter()

        for x in range(gameState.data.layout.width):
            for y in range(gameState.data.layout.height):
                if gameState.hasWall(x,y):
                    self.belief[(x,y)] = 0
                else:
                    self.belief[(x,y)] = 1

        self.belief.normalize()


    def observe(self,gameState):
        enemyPos = gameState.getAgentPosition(self.enemyIndex)
        myPos = gameState.getAgentPosition(self.index)
        noiseDistance = gameState.getAgentDistances()[self.enemyIndex]

        # noiseDistance[self.enemyIndex] = gameState.getAgentDistances(self.enemyIndex)

        if enemyPos!=None:
            for pos in self.belief:
                self.belief[pos] = 0
            self.belief[enemyPos] = 1
        else:
            for pos in self.belief:
                distance = util.manhattanDistance(myPos,pos)

                "get the probability of a noisy distance given the true distance"
                self.belief[pos] = self.belief[pos]*gameState.getDistanceProb(distance,noiseDistance)
        self.belief.normalize()

        if self.belief.totalCount() ==0:
            for x in range(gameState.data.layout.width):
                for y in range(gameState.data.layout.height):
                    if gameState.hasWall(x, y):
                        self.belief[(x, y)] = 0
                    else:
                        self.belief[(x, y)] = 1

            self.belief.normalize()


