package tileworld.agent;

import sim.engine.Schedule;
import sim.field.grid.ObjectGrid2D;
import sim.util.Bag;
import sim.util.Int2D;
import sim.util.IntBag;
import tileworld.Parameters;
import tileworld.environment.*;

import java.util.*;

public class TWAgentAreaMemory extends TWAgentWorkingMemory {
    private Schedule schedule;
    private TWAgent me;
    private final static int MAX_TIME = Parameters.lifeTime;//need to change
    private final static float MEM_DECAY = 0.5f;
    private ObjectGrid2D memoryGrid;
    private boolean[][] gridVis; // use to check whether the assigned zone is fully explored
    protected int memorySize;
    protected TWAgentPercept[][] objects;
    protected Int2D fuelStation;
    protected TWAgentPercept[][] sensedMemory;
    protected HashMap<Class<?>, TWEntity> closestInSensorRange;
    static protected List<Int2D> spiral = new NeighbourSpiral(Parameters.defaultSensorRange * 4).spiral();

    public TWAgentAreaMemory(TWAgent moi, Schedule schedule, int x, int y) {
        super(moi, schedule, x, y);
        this.me = moi;
        this.objects = new TWAgentPercept[x][y];
        this.schedule = schedule;
        this.memoryGrid = new ObjectGrid2D(me.getEnvironment().getxDimension(), me.getEnvironment().getyDimension());
        int sensorRange = Parameters.defaultSensorRange * 2 + 1;
        this.sensedMemory = new TWAgentPercept[sensorRange][sensorRange];
        this.gridVis = new boolean[me.getEnvironment().getxDimension()][me.getEnvironment().getyDimension()];
    }

    /**
     * if not object, return 0; else return the remaining time
     * @param o
     * @param threshold
     * @return
     */
    public double getEstimatedRemainLife(TWEntity o, double threshold) {
        if (objects[o.getX()][o.getY()] == null)
            return 0;
        else
            return (Parameters.lifeTime * threshold) - (this.schedule.getTime() - objects[o.getX()][o.getY()].getT());
    }

    public boolean isZoneAllVisited(int x_start,int x_end, int y_start,int y_end){
        for (int i=x_start; i<x_end; i++){
            for (int j=y_start; j<y_end;j++){
                if (this.gridVis[i][j]==false){
                    return false;
                }
            }
        }
        return true;
    }

    //get the nearby indicated objects in the zone
    protected PriorityQueue<TWEntity> getObjectsInZone(Int2D[] coordinates, Class<?> type) {
        TWObject object = null;
        // rank by distance
        PriorityQueue<TWEntity> queue = new PriorityQueue<>(10, new Comparator<TWEntity>() {
            @Override
            public int compare(TWEntity o1, TWEntity o2) {
                return (int) (((CMDAgent) me).getDistanceTo(o1) - ((CMDAgent) me).getDistanceTo(o2));
            }
        });
        //
        for (int i = coordinates[0].x; i <= coordinates[1].x; i++) {
            for (int j = coordinates[0].y; j <= coordinates[3].y; j++) {
                if (me.getEnvironment().isInBounds(i, j) && objects[i][j] != null &&
                        !(objects[i][j].getO() instanceof TWFuelStation)) {
                    object = (TWObject) objects[i][j].getO();//get the object
                    if (type.isInstance(object)){//if object is the type we want, put it into queue
                        queue.add(object);
                    }
                }
            }
        }
        return queue;
    }

    @Override
    public void updateMemory(Bag sensedObjects, IntBag objectXCoords, IntBag objectYCoords, Bag sensedAgents, IntBag agentXCoords, IntBag agentYCoords) {
//        super.updateMemory(sensedObjects, objectXCoords, objectYCoords, sensedAgents, agentXCoords, agentYCoords);
        closestInSensorRange = new HashMap<Class<?>, TWEntity>(4);

        //must all be same size.
        assert (sensedObjects.size() == objectXCoords.size() && sensedObjects.size() == objectYCoords.size());

        //        me.getEnvironment().getMemoryGrid().clear();  // THis is equivalent to only having sensed area in memory
        //       this.decayMemory();       // You might want to think about when to call the decay function as well.

        //update the sensor range memory
        int sensor_start_x = me.getX() - Parameters.defaultSensorRange;
        int sensor_start_y = me.getY() - Parameters.defaultSensorRange;
        //1- delete(reset) the sensor range memory to null
        for (int i=0;i<=Parameters.defaultSensorRange*2;i++){
            for (int j=0;j<=Parameters.defaultSensorRange*2;j++){
                // clear the sensedMemory
                if (sensedMemory[i][j]!=null){
                    sensedMemory[i][j] = null;
//                    memorySize--;
                }
                int tmp_x = sensor_start_x + i;
                int tmp_y = sensor_start_y + j;
                if(tmp_x >= 0 && tmp_x<objects.length && tmp_y>=0 && tmp_y<objects[1].length){
                    // set based on previous memory
                    sensedMemory[i][j] = objects[tmp_x][tmp_y];
                    // re-set the sensor range memory to null
                    objects[tmp_x][tmp_y] = null;
                    memoryGrid.set(tmp_x,tmp_y,null);
                    // set the grid to visited
                    gridVis[tmp_x][tmp_y] = true;
                }
            }
        }
        //2- update the memory according to the sensor observation
        for (int i = 0; i < sensedObjects.size(); i++) {
            TWEntity o = (TWEntity) sensedObjects.get(i);
            if(!(o instanceof TWEntity)){
                continue;
            }
            /**
             * find the fuel station
             */
            if(fuelStation == null && o instanceof TWFuelStation){
                fuelStation = new Int2D(o.getX(),o.getY());
            }
            int sensor_x = o.getX() - sensor_start_x;
            int sensor_y = o.getY() - sensor_start_y;
            //check whether the (same) object is already exited
            TWAgentPercept preObj = sensedMemory[sensor_x][sensor_y];
            // if the same object: reuse its original lifetime
            if (sensedMemory[sensor_x][sensor_y]!=null && preObj.getO().hashCode()==o.hashCode()){
                // update the objects and memoryGrid
                objects[o.getX()][o.getY()] = new TWAgentPercept(o, preObj.getT());
                memoryGrid.set(o.getX(),o.getY(),o);
//                memorySize ++;
            }
            //if not the same object, mark it as the new observed object(create the new lifetime)
            else {
                objects[o.getX()][o.getY()] = new TWAgentPercept(o,this.getSimulationTime());
                memoryGrid.set(o.getX(),o.getY(),o);
                memorySize ++;
            }

            //if nothing in memory currently, then were increasing the number
            //of items we have in memory by 1
            //if(objects[objectXCoords.get(i)][objectYCoords.get(i)] == null) memorySize++;
//            if(objects[o.getX()][o.getY()] == null) memorySize++;
//
//            //Add the object to memory
//            objects[o.getX()][o.getY()] = new TWAgentPercept(o, this.getSimulationTime());
//
//            memoryGrid.set(o.getX(), o.getY(), o);

            updateClosest(o);

        }
    }

    @Override
    public void updateMemory(TWEntity[][] sensed, int xOffset, int yOffset) {
//        super.updateMemory(sensed, xOffset, yOffset);
        for (int x = 0; x < sensed.length; x++) {
            for (int y = 0; y < sensed[x].length; y++) {
                objects[x + xOffset][y + yOffset] = new TWAgentPercept(sensed[x][y], this.getSimulationTime());
            }
        }
    }

    @Override
    public void decayMemory() {
        super.decayMemory();
    }

    @Override
    public void removeAgentPercept(int x, int y) {
        objects[x][y] = null;
    }

    @Override
    public void removeObject(TWEntity o) {
        removeAgentPercept(o.getX(), o.getY());
    }

    @Override
    public TWTile getNearbyTile(int x, int y, double threshold) {
        return (TWTile) this.getNearbyObject(x, y, threshold, TWTile.class);
    }

    @Override
    public TWHole getNearbyHole(int x, int y, double threshold) {
        return (TWHole) this.getNearbyObject(x, y, threshold, TWHole.class);

    }

    @Override
    public int getMemorySize() {
//        return super.getMemorySize();
        return memorySize;
    }

    @Override
    public TWEntity getClosestObjectInSensorRange(Class<?> type) {
        return closestInSensorRange.get(type);
    }

    @Override
    public boolean isCellBlocked(int tx, int ty) {
        //no memory at all, so assume not blocked
        if (objects[tx][ty] == null) {
            return false;
        }

        TWEntity e = (TWEntity) objects[tx][ty].getO();
        //is it an obstacle?
        return (e instanceof TWObstacle);
    }

    @Override
    public ObjectGrid2D getMemoryGrid() {
        return this.memoryGrid;
    }
    private double getSimulationTime() {
        return schedule.getTime();
    }
    public Int2D getFuelStation() {
        return fuelStation;
    }
    private void updateClosest(TWEntity o) {
        assert (o != null);
        if (closestInSensorRange.get(o.getClass()) == null || me.closerTo(o, closestInSensorRange.get(o.getClass()))) {
            closestInSensorRange.put(o.getClass(), o);
        }
    }

    private TWObject getNearbyObject(int sx, int sy, double threshold, Class<?> type) {

        //If we cannot find an object which we have seen recently, then we want
        //the one with maxTimestamp
        double maxTimestamp = 0;
        TWObject o = null;
        double time = 0;
        TWObject ret = null;
        int x, y;
        for (Int2D offset : spiral) {
            x = offset.x + sx;
            y = offset.y + sy;

            if (me.getEnvironment().isInBounds(x, y) && objects[x][y] != null) {
                o = (TWObject) objects[x][y].getO();//get mem object
                if (type.isInstance(o)) {//if it's not the type we're looking for do nothing

                    time = objects[x][y].getT();//get time of memory

                    if (this.getSimulationTime() - time <= threshold) {
                        //if we found one satisfying time, then return
                        return o;
                    } else if (time > maxTimestamp) {
                        //otherwise record the timestamp and the item in case
                        //it's the most recent one we see
                        ret = o;
                        maxTimestamp = time;
                    }
                }
            }
        }

        //this will either be null or the object of Class type which we have
        //seen most recently but longer ago than now-threshold.
        return ret;
    }

}
