#include "DataReader.h"
#include <time.h>

#define NUMBER_OF_BLOCKS 12
#define NUMBER_OF_CHANNELS 16

/// <summary>
/// Initializes a new instance of the <see cref="DataReader"/> class.
/// LaserIpEndPoint apunta a la direcci�n ip del laser (O a cualquier IP) escuchando solo al puerto 2368
/// ClientLIDAR es el sockect que se utiliza para la comunicaci�n con el LIDAR.
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
/// <para>Funci�n que lee el stream de bytes del LIDAR y obtiene las propiedades del punto retornadas, as� como se calculan aquellas
/// que pueden inducirse de estas primeras. Con esta inforaci�n se construye y almacena el punto Cada paquete consta de 1206 bytes
/// ClientLIDAR elimina la cabecera de 42 bytes </para>
/// </summary>
void DataReader::ReadData()
{
	int azimuth_index = 0, distance_index = 0, intensity_index = 0, fallos = 0;
	clock_t start_procces, finish, start_loop;
	cliext::vector<Punto3D ^>^ pointCloud = gcnew cliext::vector<Punto3D ^>;
	cli::array<Byte>^ ReceiveBytes;
	cli::array<Double>^ azimuths;
	cli::array<Double>^ distances;
	cli::array<Double>^ intensities;

	while (true) {
		try
		{
			Console::ResetColor();
			start_loop = clock();

			ReceiveBytes = ClientLIDAR->Receive(LaserIpEndPoint);

			start_procces = clock();

			azimuths = InterpolateAzimuth(ReceiveBytes);
			distances = ExtractDistances(ReceiveBytes);
			intensities = ExtractIntensities(ReceiveBytes);

			for (int block = 0; block < NUMBER_OF_BLOCKS * 2; block++) {
				for (int i = 0; i < NUMBER_OF_CHANNELS;i++) {
					pointCloud->push_back(gcnew Punto3D(distances[distance_index], intensities[intensity_index], azimuths[azimuth_index], getAngle(i)));
					distance_index++;
					intensity_index++;
				}
				azimuth_index++;
			}
			//TODO: Enviar vector.
			finish = clock();
			saveProcessTime((finish - start_procces) / CLOCKS_PER_SEC);
			savePackageTime((finish - start_loop) / CLOCKS_PER_SEC);

			azimuth_index = 0, distance_index = 0, intensity_index = 0, fallos = 0;;
			Console::Write("\r|\t{0}\t|\t{1}\t|\t{2}\t|\t{3}\t|\r", getProcessTime(), 60 / getProcessTime(), pointCloud->size(), getPackageTime());
		}//Try
		catch (Exception^ e)
		{
			fallos++;
			Console::BackgroundColor = ConsoleColor::Red;
			Console::Write("\r [" + fallos + "]" + e->Message);
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
		throw gcnew Exception("Paquete con formato extra�o");

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
		throw gcnew Exception("Paquete con formato extra�o");

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
		throw gcnew Exception("Paquete con formato extra�o");

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