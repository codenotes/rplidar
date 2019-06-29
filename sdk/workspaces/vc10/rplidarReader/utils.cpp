#define _WIN32_DCOM
#include <iostream>
#include <comdef.h>
#include <Wbemidl.h>
#include <vector>
#include <string>
#include "C:/usr/include/rplidar/RPlidarNeedfullsForDLL.h"

#include "C:\usr\include\GregUtils\strguple.h"

using namespace std;
#pragma comment(lib, "wbemuuid.lib")

extern "C"
{


	DLL_EXPORT void DestroyGetComPortLidar(rp::RplidarProxy::wmiReportType  ** p)
	{
		if (p != nullptr)
			delete *p,*p=nullptr;
	}

	//int main_(int argc, char **argv)
	DLL_EXPORT void GetComPortLidar(rp::RplidarProxy::wmiReportType ** pResults)
	{
		HRESULT hres;

		*pResults = new rp::RplidarProxy::wmiReportType();

		// Step 1: --------------------------------------------------
		// Initialize COM. ------------------------------------------

		hres = CoInitializeEx(0, COINIT_MULTITHREADED);
		if (FAILED(hres))
		{
			SGUP_ODSA(__FUNCTION__, "Failed to initialize COM library. Error code =", hres);

			//	<< hex << hres << endl;
			//return 1;                  // Program has failed.
		}

		// Step 2: --------------------------------------------------
		// Set general COM security levels --------------------------

		hres = CoInitializeSecurity(
			NULL,
			-1,                          // COM authentication
			NULL,                        // Authentication services
			NULL,                        // Reserved
			RPC_C_AUTHN_LEVEL_DEFAULT,   // Default authentication 
			RPC_C_IMP_LEVEL_IMPERSONATE, // Default Impersonation  
			NULL,                        // Authentication info
			EOAC_NONE,                   // Additional capabilities 
			NULL                         // Reserved
		);


		if (FAILED(hres))
		{
			SGUP_ODSA(__FUNCTION__, "Failed to initialize security. Error code =", hres);
			//cout << "Failed to initialize security. Error code = 0x"
			//	<< hex << hres << endl;
			CoUninitialize();
			//return 1;                    // Program has failed.
			//return;
		}

		// Step 3: ---------------------------------------------------
		// Obtain the initial locator to WMI -------------------------

		IWbemLocator *pLoc = NULL;

		hres = CoCreateInstance(
			CLSID_WbemLocator,
			0,
			CLSCTX_INPROC_SERVER,
			IID_IWbemLocator, (LPVOID *)&pLoc);

		if (FAILED(hres))
		{
			SGUP_ODSA(__FUNCTION__, "failed to create IWbemLocator object =", hres);
		/*	cout << "Failed to create IWbemLocator object."
				<< " Err code = 0x"
				<< hex << hres << endl;*/
			CoUninitialize();
			return;
			//return 1;                 // Program has failed.
		}

		// Step 4: -----------------------------------------------------
		// Connect to WMI through the IWbemLocator::ConnectServer method

		IWbemServices *pSvc = NULL;

		// Connect to the root\cimv2 namespace with
		// the current user and obtain pointer pSvc
		// to make IWbemServices calls.
		hres = pLoc->ConnectServer(
			_bstr_t(L"ROOT\\CIMV2"), // Object path of WMI namespace
			NULL,                    // User name. NULL = current user
			NULL,                    // User password. NULL = current
			0,                       // Locale. NULL indicates current
			NULL,                    // Security flags.
			0,                       // Authority (for example, Kerberos)
			0,                       // Context object 
			&pSvc                    // pointer to IWbemServices proxy
		);

		if (FAILED(hres))
		{

			SGUP_ODSA(__FUNCTION__, "could not connect. Error code =", hres);

			/*cout << "Could not connect. Error code = 0x"
				<< hex << hres << endl;*/
			pLoc->Release();
			CoUninitialize();
			return;                // Program has failed.
		}

		//cout << "Connected to ROOT\\CIMV2 WMI namespace" << endl;


		// Step 5: --------------------------------------------------
		// Set security levels on the proxy -------------------------

		hres = CoSetProxyBlanket(
			pSvc,                        // Indicates the proxy to set
			RPC_C_AUTHN_WINNT,           // RPC_C_AUTHN_xxx
			RPC_C_AUTHZ_NONE,            // RPC_C_AUTHZ_xxx
			NULL,                        // Server principal name 
			RPC_C_AUTHN_LEVEL_CALL,      // RPC_C_AUTHN_LEVEL_xxx 
			RPC_C_IMP_LEVEL_IMPERSONATE, // RPC_C_IMP_LEVEL_xxx
			NULL,                        // client identity
			EOAC_NONE                    // proxy capabilities 
		);

		if (FAILED(hres))
		{
			SGUP_ODSA(__FUNCTION__, "Could not set proxy blanket. Error code =", hres);

			//cout << "Could not set proxy blanket. Error code = 0x"
			//	<< hex << hres << endl;
			pSvc->Release();
			pLoc->Release();
			CoUninitialize();
			return;               // Program has failed.
		}

		// Step 6: --------------------------------------------------
		// Use the IWbemServices pointer to make requests of WMI ----

		// For example, get the name of the operating system
		IEnumWbemClassObject* pEnumerator = NULL;
		hres = pSvc->ExecQuery(
			bstr_t("WQL"),
			//bstr_t("SELECT * FROM Win32_OperatingSystem"),
			//bstr_t("Select * from Win32_SerialPort"), //does not return complete set
			bstr_t("SELECT * FROM Win32_PnPEntity WHERE ClassGuid = \"{4d36e978-e325-11ce-bfc1-08002be10318}\""),
			WBEM_FLAG_FORWARD_ONLY | WBEM_FLAG_RETURN_IMMEDIATELY,
			NULL,
			&pEnumerator);

		if (FAILED(hres))
		{
			SGUP_ODSA(__FUNCTION__, "Query for operating system name failed. Error code =", hres);

		/*	cout << "Query for operating system name failed."
				<< " Error code = 0x"
				<< hex << hres << endl;*/
			pSvc->Release();
			pLoc->Release();
			CoUninitialize();
			return;               // Program has failed.
		}

		// Step 7: -------------------------------------------------
		// Get the data from the query in step 6 -------------------

		IWbemClassObject *pclsObj = NULL;
		ULONG uReturn = 0;

		while (pEnumerator)
		{
			HRESULT hr = pEnumerator->Next(WBEM_INFINITE, 1,
				&pclsObj, &uReturn);

			if (0 == uReturn)
			{
				break;
			}

			VARIANT vtProp;

			// Get the value of the Name property
			hr = pclsObj->Get(L"DeviceID", 0, &vtProp, 0, 0);
		//	wcout << " DeviceID : " << vtProp.bstrVal << endl;
			_bstr_t name= vtProp.bstrVal;
			VariantClear(&vtProp);

			hr = pclsObj->Get(L"Name", 0, &vtProp, 0, 0);
			//wcout << " Name : " << vtProp.bstrVal << endl;
			_bstr_t desc = vtProp.bstrVal;
			VariantClear(&vtProp);

			(**pResults).emplace_back( name.operator char *(),desc.operator char *() );
			

			pclsObj->Release();
		}

		// Cleanup
		// ========

		pSvc->Release();
		pLoc->Release();
		pEnumerator->Release();
		CoUninitialize();

		return;   // Program successfully completed.

	}



}
