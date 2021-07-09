

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>

/* Constant definitions */
#define MAX_NAME_SIZE 1000
#define MAX_STD_SIZE 30

/* Student record */
typedef struct
{
	uint32_t id;
	char name[MAX_NAME_SIZE];
	uint32_t passing_year;
} Student;

int sort_by_name(const void* a, const void* b)
{
	const Student *sa = a;
	const Student *sb = b;

	int lena = strlen(sa->name);
	int lenb = strlen(sb->name);

	if (lena == lenb)
	{
		/* Names having same length */

		for (int i = 0; i < lena; i++)
		{
			if (sa->name[i] != sb->name[i])
			{
				return sa->name[i] - sb->name[i];
			}
		}

		/* Names are same */
		return 0;
	}

	/* Consider the name with less character to be the lesser of two objects */

	return lena - lenb;
}

int sort_by_passing_year(const void* a, const void* b)
{
	const Student *sa = a;
	const Student *sb = b;

	return sa->passing_year - sb->passing_year;
}

int main()
{
	/* Initialize seed for pseudo random number generator */
	srand(time(0));

	/* Database of students */
	Student std_db[MAX_STD_SIZE];

	/* Initialize everything to 0 */
	memset(std_db, 0, sizeof(std_db));

	/* Randomly populate students */
	for (uint32_t i = 0; i < MAX_STD_SIZE; i++)
	{
		/* Assign unique id */
		std_db[i].id = i + 1;

		/* Randomly populate name */
		int max_name_size = (MAX_NAME_SIZE < 10 ? MAX_NAME_SIZE : 10);
		int name_size = rand() % max_name_size + 5;
		for (int j = 0; j < name_size; j++)
		{
			std_db[i].name[j] = rand() % 26 + 'a';
		}

		/* Randomly populate passing year
		from 1999 to 2018 */
		std_db[i].passing_year = 2018 - rand() % 20;
	}

	/* Sort by name */

	qsort(std_db, MAX_STD_SIZE, sizeof(std_db[0]), sort_by_name);

	printf("Students sorted by name:\n\n");
	for (int i = 0; i < MAX_STD_SIZE; i++)
	{
		printf("%5d %5d %s\n", std_db[i].id, std_db[i].passing_year, std_db[i].name);
	}

	/* Sort by passing year */

	qsort(std_db, MAX_STD_SIZE, sizeof(std_db[0]), sort_by_passing_year);

	printf("\nStudents sorted by passing year:\n\n");
	for (int i = 0; i < MAX_STD_SIZE; i++)
	{
		printf("%5d %5d %s\n", std_db[i].id, std_db[i].passing_year, std_db[i].name);
	}

	return 0;
}