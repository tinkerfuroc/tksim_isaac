import { describe, expect, it } from 'vitest';
import type { BenchBody, BenchRow, Flag, JobStatus, LastRender, RowState } from '../api';
import {
  applyOps, arenaMaps, clashByKey, clashText, duplicateNames, estimateText, exportText, flagTone, headerState,
  headerToggleOps, jobText, mapToShow, panTiltOps, poseKey, rangeOps, summary, viewerItems, visibleRows, yamlName,
} from './tableLogic';

const STATE: RowState = { included: false, behind: true, top: false, head: false, pan_deg: 0, tilt_deg: 20 };

function row(name: string, state: Partial<RowState> = {}, group: 'saved' | 'pasted' = 'saved',
  x = 1, y = 2, clash_m: number | null = null): BenchRow {
  return {
    key: `${group}:${name}`, group, pose: { name, x, y, yaw: 0 },
    state: { ...STATE, ...state }, clash: clash_m, clash_m, flags: [],
  };
}

const ROWS = [
  row('diningroom_table_0_0', { included: true }),
  row('diningroom_destination', { included: true, behind: false, top: true }),
  row('kitchen_table_0_0'),
  row('livingroom_destination', { included: true }),
];

describe('headerState', () => {
  it('headerState some/all/none and disabled when no included rows', () => {
    expect(headerState(ROWS, 'included')).toBe('some');
    expect(headerState(ROWS.filter((r) => r.state.included), 'included')).toBe('all');
    expect(headerState([ROWS[2]], 'included')).toBe('none');
    expect(headerState(ROWS, 'behind')).toBe('some'); // 2 of the 3 included rows
    expect(headerState(ROWS, 'head')).toBe('none');
    expect(headerState([ROWS[2]], 'behind')).toBe('disabled'); // no included rows to act on
    expect(headerState([], 'included')).toBe('disabled');
  });

  it('view columns ignore excluded rows', () => {
    // kitchen_table_0_0 has behind on but is not included: the column still reads "all".
    const rows = [row('a', { included: true, behind: true }), row('b', { included: false, behind: false })];
    expect(headerState(rows, 'behind')).toBe('all');
    const ops = headerToggleOps([row('a', { included: true, behind: false }), row('b', { behind: false })], 'behind');
    expect(ops).toEqual([{ op: 'set_rows', rows: ['saved:a'], fields: { behind: true } }]);
  });
});

describe('headerToggleOps', () => {
  it('header toggle acts only on visible rows', () => {
    const shown = visibleRows(ROWS, 'dining');
    expect(shown.map((r) => r.pose.name)).toEqual(['diningroom_table_0_0', 'diningroom_destination']);
    // Both shown rows are included, so the header reads "all" and a click clears them.
    expect(headerToggleOps(shown, 'included')).toEqual([
      { op: 'set_rows', rows: ['saved:diningroom_table_0_0', 'saved:diningroom_destination'], fields: { included: false } },
    ]);
    const after = applyOps(bench(ROWS), headerToggleOps(shown, 'included')).rows;
    expect(after.find((r) => r.pose.name === 'livingroom_destination')!.state.included).toBe(true);
    expect(after.find((r) => r.pose.name === 'kitchen_table_0_0')!.state).toEqual(ROWS[2].state);
  });

  it('some or none turns the column on; nothing to act on gives no op', () => {
    expect(headerToggleOps(ROWS, 'included')[0]).toMatchObject({ fields: { included: true } });
    expect(headerToggleOps([ROWS[2]], 'top')).toEqual([]);
  });

  it('filter is a case-insensitive substring on the name', () => {
    expect(visibleRows(ROWS, '  DESTINATION ').length).toBe(2);
    expect(visibleRows(ROWS, '').length).toBe(4);
  });
});

describe('rangeOps', () => {
  it('rangeOps covers the shift-click span', () => {
    const keys = (ops: ReturnType<typeof rangeOps>) => (ops[0] as { rows: string[] }).rows;
    const down = rangeOps(ROWS, 'saved:diningroom_destination', 'saved:livingroom_destination', true);
    expect(keys(down)).toEqual(['saved:diningroom_destination', 'saved:kitchen_table_0_0', 'saved:livingroom_destination']);
    expect(down[0]).toMatchObject({ op: 'set_rows', fields: { included: true } });
    // Upwards gives the same span.
    expect(keys(rangeOps(ROWS, 'saved:livingroom_destination', 'saved:diningroom_destination', false)))
      .toEqual(keys(down));
    // An anchor that is no longer shown: just the clicked row.
    expect(keys(rangeOps(ROWS, 'saved:gone', 'saved:kitchen_table_0_0', true))).toEqual(['saved:kitchen_table_0_0']);
  });
});

describe('summary', () => {
  it('summary counts overview once', () => {
    const rows = [
      row('a', { included: true, behind: true, head: true }),
      row('b', { included: true, behind: false, top: true }),
      row('c', { included: false, behind: true, top: true, head: true }),
    ];
    expect(summary(rows)).toEqual({ poses: 2, views: 4, seconds: 52 });
    expect(summary([row('c')])).toEqual({ poses: 0, views: 0, seconds: 40 });
  });

  it('estimate text adds minutes for long renders', () => {
    expect(estimateText(52)).toBe('about 52 s');
    expect(estimateText(400)).toBe('about 400 s (7 min)');
  });
});

function bench(rows: BenchRow[]): BenchBody {
  return {
    map: 'm', version: 3, arena: true, arm_pose: 'arm_pos_navigating', arm_poses: {}, tilt_down_sign: 1,
    rows, removed_count: 0, last_render: null, job: null, flags_error: null,
  };
}

describe('applyOps (optimistic view)', () => {
  it('applies row edits, removals, clear and arm pose without touching the input', () => {
    const b = bench([...ROWS, row('POS_X', {}, 'pasted')]);
    const out = applyOps(b, [
      { op: 'set_row', row: 'saved:kitchen_table_0_0', fields: { head: true, pan_deg: 30 } },
      { op: 'remove', row: 'saved:diningroom_destination' },
      { op: 'clear_pasted' },
      { op: 'set_arm', name: 'arm_pos_drop' },
    ]);
    expect(out.rows.map((r) => r.pose.name)).toEqual(['diningroom_table_0_0', 'kitchen_table_0_0', 'livingroom_destination']);
    expect(out.rows[1].state).toMatchObject({ head: true, pan_deg: 30 });
    expect(out.removed_count).toBe(1);
    expect(out.arm_pose).toBe('arm_pos_drop');
    expect(b.rows.length).toBe(5);
    expect(b.rows[2].state.head).toBe(false);
  });

  it('returns the same object when there is nothing to apply', () => {
    const b = bench(ROWS);
    expect(applyOps(b, [])).toBe(b);
  });
});

describe('pan/tilt set', () => {
  it('sets angles on shown, included rows with head cam on', () => {
    const rows = [row('a', { included: true, head: true }), row('b', { included: true }), row('c', { head: true })];
    expect(panTiltOps(rows, -45, 30)).toEqual([
      { op: 'set_rows', rows: ['saved:a'], fields: { pan_deg: -45, tilt_deg: 30 } },
    ]);
    expect(panTiltOps([rows[1]], 0, 0)).toEqual([]);
  });
});

describe('clashes', () => {
  it('both rows of a same-name pair get the distance', () => {
    const rows = [
      row('diningroom_table_2_0', {}, 'saved', 12.835, 5.45),
      row('POS_DININGROOM_TABLE_2_0', {}, 'pasted', 12.8, 5.45, 0.03500000000000014),
      row('POS_REGISTER', {}, 'pasted'),
    ];
    const c = clashByKey(rows);
    expect(c.get('saved:diningroom_table_2_0')).toBeCloseTo(0.035);
    expect(c.get('pasted:POS_DININGROOM_TABLE_2_0')).toBeCloseTo(0.035);
    expect(c.has('pasted:POS_REGISTER')).toBe(false);
    expect(clashText(0.03500000000000014)).toBe('same name · 4 cm apart');
    expect(clashText(0.001)).toBe('same name · under 1 cm apart');
    expect(poseKey('POS_DININGROOM_TABLE_2_0')).toBe('diningroom_table_2_0');
  });

  it('duplicate names in an export are counted', () => {
    const rows = [row('kitchen', { included: true }), row('POS_KITCHEN', {}, 'pasted'), row('hall')];
    expect(duplicateNames(rows, 'all')).toEqual(['kitchen']);
    expect(duplicateNames(rows, 'ticked')).toEqual([]);
  });

  it('yaml export keeps Map Studio names verbatim, so only exact matches clash', () => {
    expect(yamlName('TestSpawn')).toBe('TestSpawn');
    expect(yamlName('POS_TEST_SPAWN')).toBe('test_spawn');
    const rows = [row('TestSpawn'), row('POS_TESTSPAWN', {}, 'pasted'), row('hall'), row('POS_HALL', {}, 'pasted')];
    expect(duplicateNames(rows, 'all', 'code')).toEqual(['testspawn', 'hall']);
    expect(duplicateNames(rows, 'all', 'yaml')).toEqual(['hall']);
  });

  it('export text is used only when it answers the current format and which', () => {
    const r = { format: 'code' as const, which: 'all' as const, text: 'POS_A = ...' };
    expect(exportText(r, 'code', 'all')).toBe('POS_A = ...');
    expect(exportText(r, 'yaml', 'all')).toBeNull();
    expect(exportText(r, 'code', 'ticked')).toBeNull();
    expect(exportText(null, 'code', 'all')).toBeNull();
    expect(exportText({ ...r, text: '' }, 'code', 'all')).toBe('');
  });
});

describe('flags and jobs', () => {
  const flag = (kind: Flag['kind']): Flag => ({ kind, part: 'base', target: 'wall', distance_m: 0.01, text: 't' });
  it('near is amber, overlap and outside are red', () => {
    expect(flagTone(flag('near'))).toBe('near');
    expect(flagTone(flag('overlap'))).toBe('bad');
    expect(flagTone(flag('outside'))).toBe('bad');
  });

  const job = (p: Partial<JobStatus>): JobStatus => ({
    id: 'j', map: 'm', status: 'running', queued_at: 0, started_at: 0, finished_at: null,
    done_views: 3, total_views: 9, position: null, message: '', ...p,
  });
  it('job status lines', () => {
    expect(jobText(job({}))).toEqual({ text: 'Rendering · 3 of 9 views', tone: 'run' });
    expect(jobText(job({ status: 'queued', position: 2 }))).toEqual({ text: 'Queued · position 2', tone: 'run' });
    expect(jobText(job({ status: 'timeout', message: 'killed after 300 s' })))
      .toEqual({ text: 'Render timeout: killed after 300 s', tone: 'bad' });
    expect(jobText(job({ status: 'interrupted', message: '' }))).toEqual({ text: 'Render interrupted', tone: 'bad' });
    expect(jobText(job({ status: 'failed', message: 'boom' }))).toEqual({ text: 'Render failed: boom', tone: 'bad' });
    expect(jobText(job({ status: 'done' }))).toBeNull();
    expect(jobText(null)).toBeNull();
  });
});

describe('viewerItems', () => {
  it('overview first, then every view of every pose in order', () => {
    const last: LastRender = {
      job_id: 'j', finished_at: 0, arm_pose: 'arm_pos_navigating', overview_url: '/renders/m/j/overview.jpg',
      items: [
        { key: 'saved:a', name: 'a', flags: [], views: [
          { view: 'behind', url: '/a-b.jpg', caption: 'behind-left' },
          { view: 'head', url: '/a-h.jpg', caption: 'head cam · pan 0° · tilt 20°' },
        ] },
        { key: 'pasted:POS_B', name: 'POS_B', flags: [flag2()], views: [{ view: 'top', url: '/b-t.jpg', caption: 'top-down' }] },
      ],
    };
    const items = viewerItems(last);
    expect(items.map((i) => i.url)).toEqual(['/renders/m/j/overview.jpg', '/a-b.jpg', '/a-h.jpg', '/b-t.jpg']);
    expect(items[0]).toMatchObject({ title: 'Overview', caption: 'top-down, rendered poses labeled' });
    expect(items[3]).toMatchObject({ title: 'POS_B', note: 'base overlaps wall by 3 cm' });
    expect(viewerItems({ ...last, overview_url: null })[0].url).toBe('/a-b.jpg');
    expect(viewerItems(null)).toEqual([]);
  });
});

function flag2(): Flag {
  return { kind: 'overlap', part: 'base', target: 'wall', distance_m: -0.03, text: 'base overlaps wall by 3 cm' };
}

describe('map list', () => {
  it('lists only Map Studio maps that have a Map Check arena', () => {
    expect(arenaMaps(null)).toEqual([]);
    expect(arenaMaps({ studio: ['a', 'b', 'c'], arenas: ['c', 'a', 'sim_only'] })).toEqual(['a', 'c']);
    expect(arenaMaps({ studio: ['a'], arenas: [] })).toEqual([]);
  });

  it('keeps the stored map while listed, else falls back to the first', () => {
    expect(mapToShow(['a', 'c'], 'c')).toBe('c');
    expect(mapToShow(['a', 'c'], 'b')).toBe('a'); // stored map lost its arena
    expect(mapToShow(['a', 'c'], null)).toBe('a');
    expect(mapToShow([], 'b')).toBeNull();
  });
});
