using namespace System;
using namespace System::IO;
using namespace System::Collections;
using namespace System::Runtime::Serialization::Formatters::Binary;
using namespace System::Runtime::Serialization;

ref class Serialization
{
public:
	static void serialize()
	{
		short some[] = {1,2,3,4,5};
		Object someO = (Object)some;

		// To serialize the hashtable (and its keys/values),  
		// you must first open a stream for writing. 
		// In this case we will use a file stream.
		FileStream^ fs = gcnew FileStream( "DataFile", FileMode::Create );

		// Construct a BinaryFormatter and use it to serialize the data to the stream.
		BinaryFormatter^ formatter = gcnew BinaryFormatter;
		try
		{
			formatter->Serialize(fs, some );
		}
		catch ( SerializationException^ e ) 
		{
			Console::WriteLine( "Failed to serialize. Reason: {0}", e->Message );
			throw;
		}
		finally
		{
			fs->Close();
		}
	}

};