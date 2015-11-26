#include "DataReader.h"
#include <time.h>

#define NUMBER_OF_BLOCKS 12
#define NUMBER_OF_CHANNELS 16

/// <summary>
/// Initializes a new instance of the <see cref="DataReader"/> class.
/// LaserIpEndPoint apunta a la dirección ip del laser (O a cualquier IP) escuchando solo al puerto 2368
/// ClientLIDAR es el sockect que se utiliza para la comunicación con el LIDAR.
/// </summary>
/// <param name="LIp">The Lidar IP.</param>
DataReader::DataReader(IPEndPoint^ LIp)//SerialPort^ p
{
	LaserIpEndPoint = LIp;
	ClientLIDAR = gcnew UdpClient(LIp);
}

/// <summary>
/// Finalizes an instance of the <see cref="DataReader"/> class.
/// </summary>
DataReader::~DataReader()
{
	delete ClientLIDAR;
	delete LaserIpEndPoint;
}

/// <summary>
/// Reads the stream data from the LIDAR.
/// <para>Función que lee el stream de bytes del LIDAR y obtiene las propiedades del punto retornadas, así como se calculan aquellas
/// que pueden inducirse de estas primeras. Con esta inforación se construye y almacena el punto Cada paquete consta de 1206 bytes
/// ClientLIDAR elimina la cabecera de 42 bytes </para>
/// </summary>

void DataReader::ReadData(cli::array<Object^> ^ data)
{
	parameters_in = (cli::array<Object^>^)data;
	if (!thread_reader || thread_reader->ThreadState != System::Threading::ThreadState::Running){
		thread_reader = gcnew Thread(gcnew ThreadStart(this, &DataReader::ReadDataThread));
		thread_reader->Start();
		}
	
	parameters_in[12] = thread_reader->ThreadState;
}
void DataReader::StopReadData()
{
	try
	{
		parameters_in[12] = System::Threading::ThreadState::Stopped;
		thread_reader->Abort();
	}
	catch (Exception^e)
	{
		
	}
}
void DataReader::ReadDataThread()
{
	double CALIBRATE_X, CALIBRATE_Y, CALIBRATE_Z, CALIBRATE_R, CALIBRATE_P, CALIBRATE_W;
	CALIBRATE_X = Convert::ToDouble(parameters_in[0]);
	CALIBRATE_Y = Convert::ToDouble(parameters_in[1]);
	CALIBRATE_Z = Convert::ToDouble(parameters_in[2]);
	CALIBRATE_R = Convert::ToDouble(parameters_in[3]);
	CALIBRATE_P = Convert::ToDouble(parameters_in[4]);
	CALIBRATE_Y = Convert::ToDouble(parameters_in[5]);

	int azimuth_index = 0, distance_index = 0, intensity_index = 0, fallos = 0, mm = 0;
	double first_azimuth = -1;
	clock_t start_procces, frecuency_clock;
	List<Punto3D^>^ pointCloud = gcnew List<Punto3D^>();
	cli::array<Byte>^ ReceiveBytes;
	cli::array<Double>^ azimuths;
	cli::array<Double>^ distances;
	cli::array<Double>^ intensities;
	StreamWriter^ loger;
	Punto3D^ p;

	try
	{
		if ((double)parameters_in[11] == -1.0) {}
				
	}
	catch (Exception^e)
	{
		Stream^ sr = (Stream^)parameters_in[11];
		loger = gcnew StreamWriter(sr);
	}
		//StreamWriter^ fs = File::CreateText("C:\\LOGS\\" + DateTime::Now.ToString("HH-mm-ss") + ".log");
		while (true) {
			try
			{
				ReceiveBytes = ClientLIDAR->Receive(LaserIpEndPoint);

				start_procces = clock();
				frecuency_clock = clock();
				azimuths = InterpolateAzimuth(ReceiveBytes);
				distances = ExtractDistances(ReceiveBytes);
				intensities = ExtractIntensities(ReceiveBytes);

				if (first_azimuth == -1)
					first_azimuth = azimuths[0];

				for (int block = 0; block < NUMBER_OF_BLOCKS * 2; block++) {
					for (int i = 0; i < NUMBER_OF_CHANNELS;i++) {

						if (azimuths[azimuth_index] >= first_azimuth) {
							parameters_in[10] = ((clock() - frecuency_clock) / CLOCKS_PER_SEC);
								//TODO: Enviar vector.
							pointCloud->Clear();
							first_azimuth = azimuths[azimuth_index];
						}

						p = gcnew Punto3D(distances[distance_index], intensities[intensity_index], azimuths[azimuth_index], getAngle(i));
						p->CalculateCoordinates(CALIBRATE_X, CALIBRATE_Y, CALIBRATE_Z);
						pointCloud[pointCloud->Count] = p;

						if ((int)parameters_in[11] != -1) {
							loger->WriteLine(p->verCoordenadas());
							loger->Flush();
						}

						distance_index++;
						intensity_index++;
					}
					azimuth_index++;
				}
	
				parameters_in[7] = ((clock() - start_procces) / CLOCKS_PER_SEC);
				parameters_in[6] = (double)parameters_in[7] / 60;

				azimuth_index = 0, distance_index = 0, intensity_index = 0, fallos = 0;
			
			}//Try
			catch (Exception^ e)
			{
				System::Windows::Forms::MessageBox::Show(e->ToString());
			}
		}//while
	}


/// <summary>
/// Gets the process time.
/// </summary>
/// <returns>process_Time</returns>
double DataReader::getProcessTime()
{
	return process_Time;
}

/// <summary>
/// Gets the time between packages.
/// </summary>
/// <returns>package_Time</returns>
double DataReader::getPackageTime()
{
	return package_Time;
}

/// <summary>
/// Saves the process time.
/// </summary>
/// <param name="time">The time.</param>
void DataReader::saveProcessTime(double time)
{
	process_Time = time;
}

/// <summary>
/// Saves the time between packages.
/// </summary>
/// <param name="time">The time.</param>
void DataReader::savePackageTime(double time)
{
	package_Time = time;
}

/// <summary>
/// Extract and interpolates the azimuths.
/// </summary>
/// <param name="ReceiveBytes">The receive bytes.</param>
/// <returns></returns>
cli::array<Double>^ DataReader::InterpolateAzimuth(cli::array<Byte>^ &ReceiveBytes) {
	if (ReceiveBytes->Length == 0)
		throw gcnew Exception("Recibiendo 0 bytes...");

	if (ReceiveBytes->Length != 1206)
		throw gcnew Exception("Paquete con formato extraño");

	cli::array<Double>^ azimuths = gcnew cli::array<Double>(24);
	int j = 2;

	for (int i = 0; i < 24; i += 2)
	{
		azimuths[i] = (ReceiveBytes[j] + (ReceiveBytes[j + 1] << 8));
		azimuths[i] /= 100;
		j += 100;
	}

	for (int i = 1; i < 23; i += 2)
	{
		if (azimuths[(i + 1)] < azimuths[i - 1]) {
			azimuths[i + 1] += 360;
			azimuths[i] = azimuths[i - 1] + (azimuths[i + 1] - (azimuths[i - 1] / 2));
			azimuths[i + 1] -= 360;
		}
		else
			azimuths[i] = azimuths[i - 1] + ((azimuths[i + 1] - azimuths[i - 1]) / 2);
		if (azimuths[i] > 360) {
			azimuths[i] -= 360;
		}
	}

	azimuths[23] = azimuths[22] + ((azimuths[22] - azimuths[20]) / 2);

	return azimuths;
}

/// <summary>
/// Extracts the distances.
/// </summary>
/// <param name="ReceiveBytes">The receive bytes.</param>
/// <returns></returns>
cli::array<Double>^ DataReader::ExtractDistances(cli::array<Byte>^ &ReceiveBytes) {
	if (ReceiveBytes->Length == 0)
		throw gcnew Exception("Recibiendo 0 bytes...");

	if (ReceiveBytes->Length != 1206)
		throw gcnew Exception("Paquete con formato extraño");

	cli::array<Double>^ distances = gcnew cli::array<Double>(384);
	int bytes = 0;
	double dist;

	for (int j = 0; j < 12; j++)
	{
		bytes += 4;
		for (int i = 0; i < 32; i++)
		{
			dist = (ReceiveBytes[bytes] + (ReceiveBytes[bytes + 1] << 8));
			distances[j * 32 + i] = (dist * 2) / 1000;
			bytes += 3;
		}
	}

	return distances;
}

/// <summary>
/// Extracts the intensities.
/// </summary>
/// <param name="ReceiveBytes">The receive bytes.</param>
/// <returns></returns>
cli::array<Double>^ DataReader::ExtractIntensities(cli::array<Byte>^ &ReceiveBytes) {
	if (ReceiveBytes->Length == 0)
		throw gcnew Exception("Recibiendo 0 bytes...");

	if (ReceiveBytes->Length != 1206)
		throw gcnew Exception("Paquete con formato extraño");

	cli::array<Double>^ intensities = gcnew cli::array<Double>(384);
	int bytes = 0;

	for (int j = 0; j < 12; j++)
	{
		bytes += 6;
		for (int i = 0; i < 32; i += 2)
		{
			intensities[j * 32 + i] = ReceiveBytes[bytes];
			bytes += 3;
		}
	}
	return intensities;
}

/// <summary>
/// Gets the angle.
/// </summary>
/// <param name="channel">The channel.</param>
/// <returns></returns>
double DataReader::getAngle(int channel)
{
	switch (channel)
	{
	case 0: return -15;
	case 1: return 1;
	case 2: return -13;
	case 3: return 3;
	case 4: return -11;
	case 5: return 5;
	case 6: return -9;
	case 7: return 7;
	case 8: return -7;
	case 9: return 9;
	case 10: return -5;
	case 11: return 11;
	case 12: return -3;
	case 13: return 13;
	case 14: return -1;
	case 15: return 15;
	}
}