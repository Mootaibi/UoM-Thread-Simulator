/*	
	Author: Mohammed Alotaibi
	ID: 9976361
	Date: 06/12/2020
	EEEN30052 Concurrent Systems coursework assignment:
*/

//Preprocessor statements:
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <mutex>
#include <random>
#include <chrono>
#include <map>
#include <sstream>

//Global constants:
int const MAX_NUM_OF_THREADS = 6;
int const NUM_OF_SAMPLES = 50;
int const NUM_OF_LINKS = 2;

//Global variables:
std::map<std::thread::id, int> threadIDs;

//Functions:

//"RangedRand" generates a random number between min and max.
double RangedRand(double min, double max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(min, max);
    return distr(gen);
}

//"ThreadID" maps the calling threads' IDs with integer values in incrementing order.
void ThreadID(int id, std::map<std::thread::id, int>& threadId) {
    std::mutex mut;
    std::unique_lock<std::mutex> map_locker(mut);
    threadId.insert(std::make_pair(std::this_thread::get_id(), id));
}

//"ThreadSearch" returns the calling thread's mapped ID using an iterator.
int ThreadSearch(std::map<std::thread::id, int>& const threadId) {
    std::map<std::thread::id, int>::iterator it
        = threadId.find(std::this_thread::get_id());
    if (it == threadId.end())
        return -1;
    else
        return it->second;
}

//Classes:

//Class "Sensor" is the base class for all sensors.
class Sensor {
public:
    //Constructor:
    Sensor(std::string& type) 
        : sensorType(type) {}

    //Get:
    virtual double get_Value() = 0; //Interface for get_Value.
    std::string get_Type() { return sensorType; } //Returns type of sensor.

    //Print:
    void print_Counter() {
        std::stringstream msg;
        msg << sensorType << " : " << counter << std::endl;
        std::cout << msg.str();
    }   //Prints the number of times the get_Value function has been called.

protected:
    int counter = 0;    //Stores the number of times get_Value has been called.

private:
    std::string sensorType; //Stores the sensor's type.
};

//"Sensor" derived classes:
class TemperatureSensor : public Sensor {
public:
    //Constructor:
    TemperatureSensor(std::string& type)
        : Sensor(type) {}

    //Get:
    double get_Value() { 
        counter++;
        return RangedRand(10, 30);
    }   //Returns a random temperature value between 10 and 30.
};
class PressureSensor : public Sensor {
public:
    //Constructor:
    PressureSensor(std::string& type)
        : Sensor(type) {}

    //Get:
    double get_Value() {
        counter++;
        return RangedRand(95, 105);
    }   //Returns a random pressure value between 95 and 105.
};
class CapacitiveSensor : public Sensor {
public:
    //Constructor:
    CapacitiveSensor(std::string& type)
        : Sensor(type) {}

    //Get:
    double get_Value() {
        counter++;
        return RangedRand(1, 5);
    }   //Returns a random capacitive value between 1 and 5.
};

//Class "SensorData" stores all a sensor's values.
class SensorData {
public:
    //Constructor:
    SensorData(std::string type)
        : sensorType(type) {}

    //Get:
    std::string get_SensorType() { return sensorType; } //Returns sensor type.
    std::vector<double> get_SensorData() { return sensorData; } //Returns sensorData vector.

    //Add:
    void add_Data(double newData) { sensorData.push_back(newData); }    //Adds new data to the sensorData vector.

private:
    std::string sensorType; //Stores the sensor's type.
    std::vector<double> sensorData; //Stores all of the sensor's data in a vector.
};

//Class "receiver" receives and prints out all sensors' data.
class Receiver {
public:
    //Constructor:
    Receiver() {}

    //Receive:
    void receive_Data(SensorData sd) {
        std::unique_lock<std::mutex> locker(Receiver_mut);
        std::string typ = sd.get_SensorType();
        if (typ == "temperature sensor") {
            for (int i = 0; i < sd.get_SensorData().size(); i++) {
                temperatureData.insert(temperatureData.end(), sd.get_SensorData().at(i));
            }
        }
        else if (typ == "pressure sensor") {
            for (int i = 0; i < sd.get_SensorData().size(); i++) {
                pressureData.insert(pressureData.end(), sd.get_SensorData().at(i));
            }
        }
        else if (typ == "capacitive sensor") {
            for (int i = 0; i < sd.get_SensorData().size(); i++) {
                capacitiveData.insert(capacitiveData.end(), sd.get_SensorData().at(i));
            }
        }
    }   //Receives thread's sensor data and stores it in a vector based on the type of data.

    //Print:
    void print_SensorData() {
        std::cout << "Temperature data";
        for (std::vector<double>::const_iterator i = temperatureData.begin(); i != temperatureData.end(); i++) {
            std::cout << ", " << *i;
        }
        std::cout << std::endl;
        std::cout << "Pressure data";
        for (std::vector<double>::const_iterator i = pressureData.begin(); i != pressureData.end(); i++) {
            std::cout << ", " << *i;
        }
        std::cout << std::endl;
        std::cout << "Capacitive data";
        for (std::vector<double>::const_iterator i = capacitiveData.begin(); i != capacitiveData.end(); i++) {
            std::cout << ", " << *i;
        }
        std::cout << std::endl;
    }   //Prints out all of the sensor data.

private:
    std::mutex Receiver_mut;    //Mutual exclusion variable.
    std::vector<double> temperatureData;    //Vector for storing all the temperature sensor's data.
    std::vector<double> pressureData;   //Vector for storing all the pressure sensor's data.
    std::vector<double> capacitiveData; //Vector for storing all the capacitive sensor's data.
};

//Class "Link" connects threads to a receiver object.
class Link {
public:
    //Constructor:
    Link(Receiver& rec, int linkNum)
        : inUse(false), myReceiver(rec), linkId(linkNum) {}

    //Get:
    bool get_InUse() { return inUse; }  //Returns true if link is in use, false otherwise.
    int get_LinkId() { return linkId; } //Returns link ID.

    //Set:
    void set_InUse() { inUse = true; }  //Sets inUse.
    void set_Idle() { inUse = false; }  //Clears inUse.

    //Write:
    void write_ToDataLink(SensorData sd) { myReceiver.receive_Data(sd); } //Sends sensor data to receiver.

private:
    bool inUse; //Stores usage state.
    Receiver& myReceiver;   //Points to receiver.
    int linkId; //Stores link ID.
};

//Class "BC" acts as a bus controller and manages access to sensors.
class BC {
public:
    //Constructor:
    BC(std::vector<Sensor*>& sensor)
        : theSensors(sensor) {}

    //Request-Release:
    void request_BC() {
        std::unique_lock<std::mutex> locker(BC_mut);
        while (lock) {
            std::stringstream msg;
            msg << "BusController is locked, thread "
                    << ThreadSearch(threadIDs) << " is about to suspend.." << std::endl;
            std::cout << msg.str();
            BC_cv.wait(locker);
        }
        lock = true;
        std::stringstream msg;
        msg << "BusController locked by thread "
                << ThreadSearch(threadIDs) << std::endl;
        std::cout << msg.str();
    }   //Requests BC, if it's in use, suspend; print all cases.
    void release_BC() {
        std::unique_lock<std::mutex> locker(BC_mut);
        lock = false;
        std::stringstream msg;
        msg << "BusController unlocked by thread "
                << ThreadSearch(threadIDs) << std::endl;
        std::cout << msg.str();
        BC_cv.notify_one();
    }   //Release BC and wake one thread waiting on BC.

    //Get:
    double get_SensorValue(int selector) { return (*theSensors[selector]).get_Value(); }    //Returns sensor's "measurement".
    std::string get_SensorType(int selector) { return (*theSensors[selector]).get_Type(); } //Returns sensor's type.

private:
    bool lock = false;  //Stores lock's state.
    std::vector<Sensor*>& theSensors;   //Vector pointer of seensors.
    std::mutex BC_mut;  //Mutual exclusion variable.
    std::condition_variable BC_cv;  //Conditional synchronization variable.
};

//Class "LAC" acts as a link access controller and manages access to links.
class LAC {
public:
    //Contrsuctor:
    LAC(Receiver& r)
        : myReceiver(r), numOfAvailableLinks(NUM_OF_LINKS) {
        for (int i = 0; i < NUM_OF_LINKS; i++) {
            commsLinks.push_back(Link(myReceiver, i));
        }
    }   //Instantiate links and place them in a vector array.

    //Request-Release:
    Link& request_Link() {
        is_Available();
        std::unique_lock<std::mutex> locker(LAC_mut);
        for (int i = 0; i < NUM_OF_LINKS; i++) {
            if (!commsLinks[i].get_InUse()) {
                commsLinks[i].set_InUse();
                numOfAvailableLinks--;
                std::stringstream msg;
                msg << "Link " << commsLinks[i].get_LinkId()
                        << " accessed by thread " << ThreadSearch(threadIDs) << std::endl;
                std::cout << msg.str();
                return std::ref(commsLinks[i]);
            }
        }
    }   //Returns pointer to any available link.
    void release_Link(Link& releasedLink) {
        std::unique_lock<std::mutex> locker(LAC_mut);
        releasedLink.set_Idle();
        numOfAvailableLinks++;
        std::stringstream msg;
        msg << "Link " << releasedLink.get_LinkId()
                << " released by thread " << ThreadSearch(threadIDs) << std::endl;
        std::cout << msg.str();
        LAC_cv.notify_one();
    }   //Release LAC and wake one thread waiting on LAC, increment no. of available links.

private:
    //Private member functions:
    void is_Available() {
        std::unique_lock<std::mutex> locker(LAC_mut);
        while (!numOfAvailableLinks) {
            std::stringstream msg;
            msg << "All links are in use, thread "
                << ThreadSearch(threadIDs) << " is about to suspend.." << std::endl;
            std::cout << msg.str();
            LAC_cv.wait(locker);
        }
    }   //Checks if there are any available links, if there are none, suspend thread.

    Receiver& myReceiver;   //Points to receiver.
    int numOfAvailableLinks;    //Stores the number of links not in use.
    std::vector<Link> commsLinks;   //Stores a vector array of links.
    std::mutex LAC_mut; //Mutual exclusion variable.
    std::condition_variable LAC_cv; //Conditional synchronization variable.
};

//Thread function:
void run(BC& theBC, LAC& theLAC, int idx) {
    //Map thread:
    ThreadID(idx, threadIDs);

    //Instantiate SensorData vector for thread and place 
    //an instance of each sensor type in the vector.
    std::vector<SensorData*> sensorData;
    sensorData.push_back(new SensorData("temperature sensor"));
    sensorData.push_back(new SensorData("pressure sensor"));
    sensorData.push_back(new SensorData("capacitive sensor"));

    //Get NUM_OF_SAMPLES samples from the sensors at random and store
    //in SensorData vector; function perforemed in mutual exclusion.
    for (int i = 0; i < NUM_OF_SAMPLES; i++) {
        theBC.request_BC();
        int sel = (int)RangedRand(0, 2);
        int val = theBC.get_SensorValue(sel);
        (*sensorData[sel]).add_Data(val);
        theBC.release_BC();

        //Sleep for random period.
        int tim = (int)RangedRand(1, 10);
        std::this_thread::sleep_for(std::chrono::microseconds(tim));
    }

    //Write SensorData to link
    for (int i = 0; i < 3; i++) {
        Link* temp = &theLAC.request_Link();
        temp->write_ToDataLink(*sensorData[i]);
        theLAC.release_Link(*temp);

        //Sleep for random period.
        int tim = (int)RangedRand(1, 10);
        std::this_thread::sleep_for(std::chrono::microseconds(tim));
    }
}

int main(void) {
    //Sensors instantiation:
    std::vector<Sensor*> sensors;
    std::string t = "temperature sensor";
    std::string p = "pressure sensor";
    std::string c = "capacitive sensor";
    sensors.push_back(new TemperatureSensor(t));
    sensors.push_back(new PressureSensor(p));
    sensors.push_back(new CapacitiveSensor(c));

    //Controller and receiver instantiations:
    BC theBC(std::ref(sensors));
    Receiver rec;
    LAC theLAC(rec);

    //Thread instantiation:
    std::thread threads[MAX_NUM_OF_THREADS];
    for (int i = 0; i < MAX_NUM_OF_THREADS; i++) {
        threads[i] = std::thread(run, std::ref(theBC), std::ref(theLAC), i);
    }

    //Wait for threads to terminate.
    for (int i = 0; i < MAX_NUM_OF_THREADS; i++) {
        threads[i].join();
    }

    //Prints:
    rec.print_SensorData();
    std::cout << "All threads terminated" << std::endl;
    for (int i = 0; i < 3; i++) {
        (*sensors[i]).print_Counter();
    }

    //End of code.
    return 0;
}