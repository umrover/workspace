#ifndef SEARCHER_HPP
#define SEARCHER_HPP

#include "rover.hpp"

class StateMachine;

class Searcher {
public:
  /*************************************************************************/
  /* Public Member Functions */
  /*************************************************************************/
  Searcher(StateMachine* stateMachine_)
  : stateMachine(stateMachine_) {}

  virtual ~Searcher() {}
  
  NavState run( Rover * mPhoebe, const rapidjson::Document& mRoverConfig );
  
  Odometry frontSearchPoint();

  void popSearchPoint();

private:
  /*************************************************************************/
  /* Private Enum Class */
  /*************************************************************************/
  enum class SearchState
  {
    SearchFaceNorth,
    SearchTurn120,
    SearchTurn240,
    SearchTurn360,
    SearchTurn,
    SearchDrive,
    TurnToBall,
    DriveToBall
  };

  /*************************************************************************/
  /* Private Member Functions */
  /*************************************************************************/
  NavState executeSearchFaceNorth( Rover* mPhoebe );

  NavState executeSearchFace120( Rover* mPhoebe );

  NavState executeSearchFace240( Rover* mPhoebe );

  NavState executeSearchFace360( Rover* mPhoebe, const rapidjson::Document& mRoverConfig );

  NavState executeSearchTurn( Rover* mPhoebe, const rapidjson::Document& mRoverConfig );

  NavState executeSearchDrive( Rover* mPhoebe );

  NavState executeTurnToBall( Rover* mPhoebe );

  NavState executeDriveToBall( Rover* mPhoebe );

  virtual void initializeSearch( Rover* mPhoebe, const rapidjson::Document& mRoverConfig ) = 0;

protected:  
  /*************************************************************************/
  /* Protected Member Variables */
  /*************************************************************************/
  StateMachine* stateMachine;

  // Vector of search point multipliers used as a base for the search points.
  vector< pair<short, short> > mSearchPointMultipliers;

  // Queue of search points.
  queue<Odometry> mSearchPoints;

};


#endif //SEARCHER_HPP