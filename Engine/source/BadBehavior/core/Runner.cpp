#include "console/engineAPI.h"
#include "platform/profiler.h"

#include "Runner.h"


using namespace BadBehavior;

IMPLEMENT_CONOBJECT(BehaviorTreeRunner);

BehaviorTreeRunner::BehaviorTreeRunner()
   : mOwner(NULL), 
     mRootNode(NULL), 
     mRootTask(NULL) 
{}


BehaviorTreeRunner::~BehaviorTreeRunner()
{
   delete mRootTask;
}


void BehaviorTreeRunner::initPersistFields()
{
   addGroup( "Behavior" );

   addProtectedField("rootNode", TYPEID< SimObject >(), Offset(mRootNode, BehaviorTreeRunner),
      &_setRootNode, &defaultProtectedGetFn, "@brief The root node of the tree to be run.");

   addProtectedField("ownerObject", TYPEID< SimObject >(), Offset(mOwner, BehaviorTreeRunner),
      &_setOwner, &defaultProtectedGetFn, "@brief The object that owns the tree to be run.");

   endGroup( "Behavior" );

   Parent::initPersistFields();
}


// processTick is where the magic happens :)
void BehaviorTreeRunner::processTick()
{
   PROFILE_SCOPE(BehaviorTreeRunner_processTick);

   // check that we are setup to run
   if(mOwner.isNull() || mRootNode.isNull())
      return;

   if(!mRootTask)
   {
      if((mRootTask = mRootNode->createTask()) == NULL)
      {
         Con::errorf("BehaviorTreeTicker::processTick, no task for root node");
         return;
      }
   }

   if(mTasks.empty())
   {
      // init the root node
      mRootTask->setup(mOwner, this);

      // add it to the task list
      mTasks.push_back(mRootTask);
   }

   // some vars we will use
   Task *currentTask = NULL;
   Task *nextTask = NULL;
   
   // loop through the tasks in the task list
   while(mTasks.size())
   {
      // get a task
      currentTask = mTasks.back();

      // tick the task, and get its child and status
      nextTask = currentTask->tick();
      
      // if task returned no children, it has completed
      if(!nextTask)
      {
         // so remove it from the list
         mTasks.pop_back();
         if(mTasks.size())
            // and tell its parent that it completed
            mTasks.back()->onChildComplete(currentTask->getStatus());

         currentTask->finish();
      }
      else
      {
         // add the child as a task
         nextTask->setup(mOwner, this);
         mTasks.push_back(nextTask);
      }
   }  
}


bool BehaviorTreeRunner::_setRootNode( void *object, const char *index, const char *data )
{
   BehaviorTreeRunner *runner = static_cast<BehaviorTreeRunner *>( object );   
   CompositeNode* root = NULL;
   Sim::findObject( data, root );
   if(root)
      runner->setRootNode(root);
   return false;
}

bool BehaviorTreeRunner::_setOwner( void *object, const char *index, const char *data )
{
   BehaviorTreeRunner *runner = static_cast<BehaviorTreeRunner *>( object );   
   SimObject* owner = NULL;
   Sim::findObject( data, owner );
   if(owner)
      runner->setOwner(owner);
   return false;
}


void BehaviorTreeRunner::setOwner(SimObject *owner) 
{ 
   reset();
   mOwner = owner; 
}


void BehaviorTreeRunner::setRootNode(CompositeNode *root) 
{ 
   reset();
   mRootNode = root; 
}


void BehaviorTreeRunner::stop()
{
   setProcessTicks(false);
}


void BehaviorTreeRunner::start()
{
   setProcessTicks(true);
}


void BehaviorTreeRunner::reset()
{
   mTasks.clear();
   if(mRootTask)
   {
      delete mRootTask;
      mRootTask = 0;
   }
}


void BehaviorTreeRunner::clear()
{
   reset();
   mRootNode = 0;
}


bool BehaviorTreeRunner::isRunning()
{
   return isProcessingTicks();
}


DefineEngineMethod( BehaviorTreeRunner, stop, void, (), ,
                   "Halt the execution of the behavior tree.\n\n"
                   "@note The internal task status is retained, allowing execution to be resumed.")
{
   object->stop();
}


DefineEngineMethod( BehaviorTreeRunner, start, void, (), ,
                  "Resume execution of the (stopped) behavior tree.")
{
   object->start();
}


DefineEngineMethod( BehaviorTreeRunner, reset, void, (), ,
                    "Reset the behavior tree. Any internal state is lost.")
{
   object->reset();
}


DefineEngineMethod( BehaviorTreeRunner, clear, void, (), ,
                    "Clear the behavior tree.")
{
   object->clear();
}


DefineEngineMethod( BehaviorTreeRunner, isRunning, bool, (), ,
                    "Is the behavior tree running")
{
   return object->isRunning();
}