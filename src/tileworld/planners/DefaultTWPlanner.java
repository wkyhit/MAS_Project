/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package tileworld.planners;

import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.agent.TWAgent;
import tileworld.environment.TWDirection;

import java.util.ArrayList;

/**
 * DefaultTWPlanner
 *
 * @author michaellees
 * Created: Apr 22, 2010
 *
 * Copyright michaellees 2010
 *
 * Here is the skeleton for your planner. Below are some points you may want to
 * consider.
 *
 * Description: This is a simple implementation of a Tileworld planner. A plan
 * consists of a series of directions for the agent to follow. Plans are made,
 * but then the environment changes, so new plans may be needed
 *
 * As an example, your planner could have 4 distinct behaviors:
 *
 * 1. Generate a random walk to locate a Tile (this is triggered when there is
 * no Tile observed in the agents memory
 *
 * 2. Generate a plan to a specified Tile (one which is nearby preferably,
 * nearby is defined by threshold - @see TWEntity)
 *
 * 3. Generate a random walk to locate a Hole (this is triggered when the agent
 * has (is carrying) a tile but doesn't have a hole in memory)
 *
 * 4. Generate a plan to a specified hole (triggered when agent has a tile,
 * looks for a hole in memory which is nearby)
 *
 * The default path generator might use an implementation of A* for each of the behaviors
 *
 */
public class DefaultTWPlanner implements TWPlanner {
    private ArrayList<Int2D> goal_list;
    private TWPath path;
    private TWAgent agent;
    private  AstarPathGenerator pathGenerator;

    public DefaultTWPlanner(TWAgent a) {
        this.agent = a;
        this.path = null;
        this.goal_list = new ArrayList<>(0);
        this.pathGenerator = new AstarPathGenerator(this.agent.getEnvironment(),this.agent, Parameters.xDimension+Parameters.yDimension);
    }

    public TWPath generatePlan() {
        path = pathGenerator.findPath(this.agent.getX(),this.agent.getY(),goal_list.get(0).x,goal_list.get(0).y);
        return path;
    }

    public boolean hasPlan() {
        if(path!=null && path.hasNext()){
            return true;
        }
        else{
            return false;
        }
    }

    public void voidPlan() {
        path = null;
    }

    public Int2D getCurrentGoal() {
        return goal_list.isEmpty()? null : goal_list.get(0);
    }

    public ArrayList<Int2D> getGoalList() {
        return goal_list;
    }

    public TWDirection execute() {
        return path.popNext().getDirection();
    }

}

