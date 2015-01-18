//-----------------------------------------------------------------------------
// Torque Game Engine
// Written by John Vanderbeck
//
// This code is written by John Vanderbeck and is offered freely to the Torque
// Game Engine wth no express warranties.  Use it for whatever you want, all
// I ask is that you don't rip it off and call it your own.  Credit where
// credit is due.  If you do use this, just drop me a line to let me know.  It
// makes me fell good :)
// Contact: jvanderbeck@novusdelta.com
//          http://www.novusdelta.com
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// This code implements support for SQLite into Torque and TorqueScript
//
// Essentially this creates a scriptable object that interfaces with SQLite.
//
// The supported SQL subset of SQLite can be found here:
// http://www.sqlite.org/lang.html
//-----------------------------------------------------------------------------

#ifndef _SQLITEOBJECT_H_
#define _SQLITEOBJECT_H_

#ifndef _SIMBASE_H_
#include "console/simBase.h"
#endif

#include "console/sqlite3.h"
#include "core/util/tVector.h"

struct sqlite_resultrow
{
   VectorPtr<char*> vColumnNames;
   VectorPtr<char*> vColumnValues;
};

struct sqlite_resultset
{
   int                           iResultSet;
   int                           iCurrentRow;
   int                           iCurrentColumn;
   int                           iNumRows;
   int                           iNumCols;
   bool                          bValid;
   VectorPtr<sqlite_resultrow*>  vRows;
};


class SQLiteObject : public SimObject
{
   // This typedef is required for tie ins with the script language.
   //--------------------------------------------------------------------------
	protected:
      typedef SimObject Parent;
   //--------------------------------------------------------------------------

   public:
      SQLiteObject();
      ~SQLiteObject();

      // These are overloaded functions from SimObject that we handle for
      // tie in to the script language.  The .cc file has more in depth
      // comments on these.
      //-----------------------------------------------------------------------
      bool processArguments(S32 argc, const char **argv);
      bool onAdd();
      void onRemove();
      static void initPersistFields();
      //-----------------------------------------------------------------------

      //-----------------------------------------------------------------------
      // Called to open a database using the sqlite_open() function.
      // If the open fails, the function returns false, and sets the
      // global error string.  The script interface will automaticly
      // call the onOpenFailed() script callback and pass this string
      // in if it fails.  If it succeeds the script interface will call
      // the onOpened() script callback.
      bool OpenDatabase(const char* filename);
      void CloseDatabase();
      int ExecuteSQL(const char* sql);
      void NextRow(int resultSet);
      bool EndOfResult(int resultSet);

      // support functions
      void ClearErrorString();
      void ClearResultSet(int index);
      sqlite_resultset* GetResultSet(int iResultSet);
      bool SaveResultSet(sqlite_resultset* pResultSet);
      int GetResultSetIndex(int iResultSet);
      int GetColumnIndex(int iResult, const char* columnName);
	  int numResultSets();

   private:
      sqlite3*                       m_pDatabase;
      char*                         m_szErrorString;
      VectorPtr<sqlite_resultset*>  m_vResultSets;
      int                           m_iLastResultSet;
      int                           m_iNextResultSet;


   // This macro ties us into the script engine, and MUST MUST MUST be declared
   // in a public section of the class definition.  If it isn't you WILL get
   // errors that will confuse you.
   //--------------------------------------------------------------------------
   public:
   DECLARE_CONOBJECT(SQLiteObject);
   //--------------------------------------------------------------------------
};

#endif // _SQLITEOBJECT_H_